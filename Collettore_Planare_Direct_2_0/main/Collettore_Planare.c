// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "esp_netif.h"
#include "esp_wifi_netif.h"
#include "driver/uart.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "string.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <sys/fcntl.h>
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "math.h"
#include <stdlib.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_event.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include <unistd.h>
#include "esp_timer.h"
#include "esp_sleep.h"
#include "freertos/semphr.h"
#include "esp_event_loop.h"
#include "mbedtls/md5.h"

#include "cJSON.h"
#include <stdint.h>
#include <stddef.h>
//#include "protocol_examples_common.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "mqtt_client.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_sleep.h"

extern const char server_cert_pem_start[] asm("_binary_cert_pem_start");
extern const char server_cert_pem_end[] asm("_binary_cert_pem_end");

#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY
#define WEB_SERVER         CONFIG_WEB_SERVER
#define WEB_URL            CONFIG_WEB_URL
#define WEB_PORT           CONFIG_WEB_PORT
#define MQTT_URL           CONFIG_MQTT_URL

#define MD5_MAX_LEN 16
#define MEMORY_DEBUG

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

uint32_t MQTT_CONNEECTED = 0;

// for DC motors
#define LATCH_GPIO 16
#define CLOCK_GPIO 17
#define DATA_GPIO 22
#define SRCLR 21
#define POWER_GPIO 23
#define PRES 5
/*
#define GPIO_RIGHT_1 12
#define GPIO_LEFT_1 16

#define GPIO_RIGHT_2 2
#define GPIO_LEFT_2 17

#define GPIO_SENSE_IN 26
#define GPIO_SENSE_OUT 25

#define GPIO_ENABLE 18
*/
//#define GPIO_ROOT 16
//for ADC
#define DEFAULT_VREF 1100
#define NO_OF_SAMPLES 64
#define ESP_INTR_FLAG_DEFAULT 0

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_4;     
static const adc_bits_width_t width = ADC_WIDTH_BIT_12; //resolution
static const adc_atten_t atten = ADC_ATTEN_DB_0; //attenuation
static const adc_unit_t unit = ADC_UNIT_1; //ADC_UNIT_1 or ADC_UNIT_2

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
esp_netif_t *sta;
esp_event_handler_instance_t instance_any_id;
esp_event_handler_instance_t instance_got_ip;
int32_t configured = 0;
int motor_configured = 0;
int global_number_modules = 0;

static const char *TAG = "Collettore_Planare";
esp_mqtt_client_handle_t client = NULL;
static void main_routine();

static const char REQUEST[512] = "POST " WEB_URL " HTTP/1.0\r\n"
    "User-Agent: esp-idf/1.0 esp32\r\n"
    "Host: "WEB_SERVER"\r\n"
    "Content-Type: application/json\r\n"
    "Content-Length: 35\r\n"
    "\r\n"
    "{\"macAddress\": %s}";


//static int averageONValues [12] = {16,16,16,16,16,16,16,16,16,16,16,16};
//static int averageOFFValues [12] = {16,16,16,16,16,16,16,16,16,16,16,16};
static int64_t initialTime = 0;
static int max_time = 240;
static int32_t ValveStates [12] = {0,0,0,0,0,0,0,0,0,0,0,0};
static int average_time_opening[12] = {350,350,350,350,350,350,350,350,350,350,350,350};
static int average_time_closing[12] = {350,350,350,350,350,350,350,350,350,350,350,350};

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val){
    uint8_t i;
    for (i = 0; i < 8; i++){
        if(bitOrder == 0){
            //printf("Bit: %d, VALUE: %d\n",i,!!(val & (1 << i)));
            gpio_set_level(dataPin, !!(val & (1 << i)));
        }
        else{
            gpio_set_level(dataPin, !!(val & (1 << (7 - i))));
        }
        gpio_set_level(clockPin, 1);
        gpio_set_level(clockPin, 0);
    }
}

void clearShiftRegisters(){
  
  for(int i = 0; i < 12; i++){
    shiftOut(DATA_GPIO, CLOCK_GPIO, 0, 0);
  }
  gpio_set_level(LATCH_GPIO, 1);
  gpio_set_level(LATCH_GPIO, 0);

  gpio_set_level(SRCLR, 0);
  gpio_set_level(LATCH_GPIO, 1);
  gpio_set_level(LATCH_GPIO, 0);
  gpio_set_level(SRCLR, 1);
}

static int64_t timerSetting_ON(int motor_n, int value){
    printf("timer Setting ON\n");
    int count_times = 0;
    esp_err_t r;
    //uint8_t i = 0;
    int media = 10000;
    int values [] = {0,0,0,0,0};
    int index = 0;
    uint32_t voltage = 0;
    int alarm = 0;
    
    do{
        uint32_t adc_reading = 0;
        //Multisampling
        for (int j = 0; j < NO_OF_SAMPLES; j++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_config_channel_atten((adc2_channel_t)channel, atten);
                r = adc2_get_raw((adc2_channel_t)channel, width, &raw);

                if ( r == ESP_OK ) {
                printf("SINGLE RAW: %d\n",raw);
                } else if ( r == ESP_ERR_INVALID_STATE ) {
                    printf("%s: ADC2 not initialized yet.\n", esp_err_to_name(r));
                } else if ( r == ESP_ERR_TIMEOUT ) {
                    //This can not happen in this example. But if WiFi is in use, such error code could be returned.
                    printf("%s: ADC2 is in use by Wi-Fi.\n", esp_err_to_name(r));
                } else {
                    printf("%s\n", esp_err_to_name(r));
                }
                
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        //printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        //vTaskDelay(pdMS_TO_TICKS(1000));
        //i = i + 1;
        if(voltage < (float)(media * 1.3)){
            values[index] = voltage;  
            alarm = 0;  
            if(index == 4){
                index = 0;
            }else{
                index ++;
            } 
        }
        else{
            alarm++;
            printf("ALARM DETECTED: %i\n",alarm);
        }
        if(count_times > 4){
            media = (values[0] + values[1] + values[2] + values[3] + values[4]) / 5;
        }
        count_times++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    while(alarm < 3 && count_times < max_time);
    return count_times;
}

static int64_t timerSetting_OFF(void){
    printf("timer Setting OFF\n");
    int count_times = 0;
    esp_err_t r;
    //uint8_t i = 0;
    int media = 10000;
    int values [] = {0,0,0,0,0};
    int index = 0;
    uint32_t voltage = 0;
    int alarm = 0;
    do{
        uint32_t adc_reading = 0;
        //Multisampling
        for (int j = 0; j < NO_OF_SAMPLES; j++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_config_channel_atten((adc2_channel_t)channel, atten);
                r = adc2_get_raw((adc2_channel_t)channel, width, &raw);

                if ( r == ESP_OK ) {
                printf("SINGLE RAW: %d\n",raw);
                } else if ( r == ESP_ERR_INVALID_STATE ) {
                    printf("%s: ADC2 not initialized yet.\n", esp_err_to_name(r));
                } else if ( r == ESP_ERR_TIMEOUT ) {
                    //This can not happen in this example. But if WiFi is in use, such error code could be returned.
                    printf("%s: ADC2 is in use by Wi-Fi.\n", esp_err_to_name(r));
                } else {
                    printf("%s\n", esp_err_to_name(r));
                }
                
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        //printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        //vTaskDelay(pdMS_TO_TICKS(1000));
        //i = i + 1;
        if(voltage < (float)(media * 1.3)){
            values[index] = voltage;  
            alarm = 0;  
            if(index == 4){
                index = 0;
            }else{
                index ++;
            } 
        }
        else{
            alarm++;
            printf("ALARM DETECTED: %i\n",alarm);
        }
        if(count_times > 4){
            media = (values[0] + values[1] + values[2] + values[3] + values[4]) / 5;
        }
        count_times++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    while(alarm < 3 && count_times < max_time);
    
    return count_times;
}

static void readValue_off(int motor_n){ //void
    int count_times = 0;
    esp_err_t r;
    //uint8_t i = 0;
    int media = 10000;
    int values [] = {0,0,0,0,0};
    int index = 0;
    uint32_t voltage = 0;
    int alarm = 0;
    
   do{
        uint32_t adc_reading = 0;
        //Multisampling
        for (int j = 0; j < NO_OF_SAMPLES; j++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_config_channel_atten((adc2_channel_t)channel, atten);
                r = adc2_get_raw((adc2_channel_t)channel, width, &raw);

                if ( r == ESP_OK ) {
                printf("SINGLE RAW: %d\n",raw);
                } else if ( r == ESP_ERR_INVALID_STATE ) {
                    printf("%s: ADC2 not initialized yet.\n", esp_err_to_name(r));
                } else if ( r == ESP_ERR_TIMEOUT ) {
                    //This can not happen in this example. But if WiFi is in use, such error code could be returned.
                    printf("%s: ADC2 is in use by Wi-Fi.\n", esp_err_to_name(r));
                } else {
                    printf("%s\n", esp_err_to_name(r));
                }
                
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        //printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        
        if(voltage < (float)(media * 1.3)){
            values[index] = voltage;  
            alarm = 0;  
            if(index == 4){
                index = 0;
            }else{
                index ++;
            } 
        }
        else{
            alarm++;
            printf("ALARM DETECTED: %i\n",alarm);
        }
        if(count_times > 4){
            media = (values[0] + values[1] + values[2] + values[3] + values[4]) / 5;
        }
        count_times++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    while(alarm < 3 && count_times < max_time);
}

static void readValue_on(int motor_n, int value){ //void

    int count_times = 0;
    esp_err_t r;
    //uint8_t i = 0;
    int media = 10000;
    int values [] = {0,0,0,0,0};
    int index = 0;
    uint32_t voltage = 0;
    int closing_time = (average_time_opening[motor_n] * value) / 100;
    int max_close_time = (max_time * value) / 100;
    int alarm = 0;
    
    do{
        uint32_t adc_reading = 0;
        //Multisampling
        for (int j = 0; j < NO_OF_SAMPLES; j++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_config_channel_atten((adc2_channel_t)channel, atten);
                r = adc2_get_raw((adc2_channel_t)channel, width, &raw);

                if ( r == ESP_OK ) {
                printf("SINGLE RAW: %d\n",raw);
                } else if ( r == ESP_ERR_INVALID_STATE ) {
                    printf("%s: ADC2 not initialized yet.\n", esp_err_to_name(r));
                } else if ( r == ESP_ERR_TIMEOUT ) {
                    //This can not happen in this example. But if WiFi is in use, such error code could be returned.
                    printf("%s: ADC2 is in use by Wi-Fi.\n", esp_err_to_name(r));
                } else {
                    printf("%s\n", esp_err_to_name(r));
                }
                
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        //printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        
        if(voltage < (float)(media * 1.3)){
            values[index] = voltage;  
            alarm = 0;  
            if(index == 4){
                index = 0;
            }else{
                index ++;
            } 
        }
        else{
            alarm++;
            printf("ALARM DETECTED: %i\n",alarm);
        }
        if(count_times > 4){
            media = (values[0] + values[1] + values[2] + values[3] + values[4]) / 5;
        }
        count_times++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    while(alarm < 3 && count_times < closing_time && count_times < max_close_time);
}


static unsigned int OpeningValve(int motor_n, bool setup, int value){
    unsigned int result = 0;
    int32_t valveStateEEPROM;
    esp_err_t err;
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");
    }
    //int gpio_right = 0;
    //int gpio_left = 0;
    uint8_t datashiftregister = 0;
    int numShift = 0;

    switch(motor_n){
        /*
        case 0:err = nvs_get_i32(my_handle, "valveState1", &valveStateEEPROM); datashiftregister = 64; numShift = 0;break;
        case 1:err = nvs_get_i32(my_handle, "valveState2", &valveStateEEPROM); datashiftregister = 16; numShift = 0;break;
        case 2:err = nvs_get_i32(my_handle, "valveState3", &valveStateEEPROM); datashiftregister = 64; numShift = 1;break;
        case 3:err = nvs_get_i32(my_handle, "valveState4", &valveStateEEPROM); datashiftregister = 16; numShift = 1;break;
        case 4:err = nvs_get_i32(my_handle, "valveState5", &valveStateEEPROM); datashiftregister = 64; numShift = 2;break;
        case 5:err = nvs_get_i32(my_handle, "valveState6", &valveStateEEPROM); datashiftregister = 16; numShift = 2;break;
        case 6:err = nvs_get_i32(my_handle, "valveState7", &valveStateEEPROM); datashiftregister = 64; numShift = 3;break;
        case 7:err = nvs_get_i32(my_handle, "valveState8", &valveStateEEPROM); datashiftregister = 16; numShift = 3;break;
        case 8:err = nvs_get_i32(my_handle, "valveState9", &valveStateEEPROM); datashiftregister = 64; numShift = 4;break;
        case 9:err = nvs_get_i32(my_handle, "valveState10", &valveStateEEPROM); datashiftregister = 16; numShift = 4;break;
        case 10:err = nvs_get_i32(my_handle, "valveState11", &valveStateEEPROM); datashiftregister = 64; numShift = 5;break;
        case 11:err = nvs_get_i32(my_handle, "valveState12", &valveStateEEPROM); datashiftregister = 16; numShift = 5;break;
        default:break;
        */
        case 0:err = nvs_get_i32(my_handle, "valveState1", &valveStateEEPROM); datashiftregister = 32; numShift = 0;break;
        case 1:err = nvs_get_i32(my_handle, "valveState2", &valveStateEEPROM); datashiftregister = 128; numShift = 0;break;
        case 2:err = nvs_get_i32(my_handle, "valveState3", &valveStateEEPROM); datashiftregister = 32; numShift = 1;break;
        case 3:err = nvs_get_i32(my_handle, "valveState4", &valveStateEEPROM); datashiftregister = 128; numShift = 1;break;
        case 4:err = nvs_get_i32(my_handle, "valveState5", &valveStateEEPROM); datashiftregister = 32; numShift = 2;break;
        case 5:err = nvs_get_i32(my_handle, "valveState6", &valveStateEEPROM); datashiftregister = 128; numShift = 2;break;
        case 6:err = nvs_get_i32(my_handle, "valveState7", &valveStateEEPROM); datashiftregister = 32; numShift = 3;break;
        case 7:err = nvs_get_i32(my_handle, "valveState8", &valveStateEEPROM); datashiftregister = 128; numShift = 3;break;
        case 8:err = nvs_get_i32(my_handle, "valveState9", &valveStateEEPROM); datashiftregister = 32; numShift = 4;break;
        case 9:err = nvs_get_i32(my_handle, "valveState10", &valveStateEEPROM); datashiftregister = 128; numShift = 4;break;
        case 10:err = nvs_get_i32(my_handle, "valveState11", &valveStateEEPROM); datashiftregister = 32; numShift = 5;break;
        case 11:err = nvs_get_i32(my_handle, "valveState12", &valveStateEEPROM); datashiftregister = 128; numShift = 5;break;
        default:break;

        /*
        case 0:err = nvs_get_i32(my_handle, "valveState1", &valveStateEEPROM);gpio_right = GPIO_RIGHT_1; gpio_left = GPIO_LEFT_1;break;
        case 1:err = nvs_get_i32(my_handle, "valveState2", &valveStateEEPROM);gpio_right = GPIO_RIGHT_2; gpio_left = GPIO_LEFT_2;break;
        default:break;
        */
    }

    switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("Valve State variable = %d\n", valveStateEEPROM);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
    }
    //if(valveStateEEPROM == 0){
    if(valveStateEEPROM != value){   
        //attiva motore con datashiftregsiter e numshift
        shiftOut(DATA_GPIO, CLOCK_GPIO, 0, datashiftregister);
        for(int i = 0; i < numShift;i++){
            shiftOut(DATA_GPIO, CLOCK_GPIO, 0, 0);
        }
        gpio_set_level(LATCH_GPIO, 1);
        gpio_set_level(LATCH_GPIO, 0);
        
        if (setup == false){
                //readValue_off(motor_n); 
                readValue_on(motor_n,value);
        }
        else if (setup == true){
                result = (int)timerSetting_ON(motor_n,value);
                
        }

        clearShiftRegisters();

        switch(motor_n){
            case 0: printf("Setting the value of valve 1 to: %i\n",value);
                    err = nvs_set_i32(my_handle,"valveState1",value);
                    ValveStates[0] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 1: printf("Setting the valueof valve 2 to: %i\n",value);
                    err = nvs_set_i32(my_handle,"valveState2",value);
                    ValveStates[1] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 2: printf("Setting the value of valve 3 to: %i\n",value);
                    err = nvs_set_i32(my_handle,"valveState3",value);
                    ValveStates[2] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 3: printf("Setting the valueof valve 4 to: %i\n",value);
                    err = nvs_set_i32(my_handle,"valveState4",value);
                    ValveStates[3] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 4: printf("Setting the value of valve 5 to: %i\n",value);
                    err = nvs_set_i32(my_handle,"valveState5",value);
                    ValveStates[4] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 5: printf("Setting the valueof valve 6 to: %i\n",value);
                    err = nvs_set_i32(my_handle,"valveState6",value);
                    ValveStates[5] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 6: printf("Setting the value of valve 7 to: %i\n",value);
                    err = nvs_set_i32(my_handle,"valveState7",value);
                    ValveStates[6] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 7: printf("Setting the valueof valve 8 to: %i\n",value);
                    err = nvs_set_i32(my_handle,"valveState8",value);
                    ValveStates[7] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 8: printf("Setting the value of valve 9 to: %i\n",value);
                    err = nvs_set_i32(my_handle,"valveState9",value);
                    ValveStates[8] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 9: printf("Setting the valueof valve 10 to: %i\n",value);
                    err = nvs_set_i32(my_handle,"valveState10",value);
                    ValveStates[9] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;  
            case 10: printf("Setting the value of valve 11 to: %i\n",value);
                    err = nvs_set_i32(my_handle,"valveState11",value);
                    ValveStates[10] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 11: printf("Setting the valueof valve 12 to: %i\n",value);
                    err = nvs_set_i32(my_handle,"valveState12",value);
                    ValveStates[11] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;              
            
            default:break;
        } 
     
    }
    nvs_close(my_handle);
    return result;
}

static unsigned int ClosingValve(int motor_n, bool setup){
    unsigned int result = 0;
    int32_t valveStateEEPROM;
    esp_err_t err;
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");
    }

    uint8_t datashiftregister = 0;
    int numShift = 0;
    switch(motor_n){
        /*
        case 0:err = nvs_get_i32(my_handle, "valveState1", &valveStateEEPROM); datashiftregister = 128; numShift = 0;break;
        case 1:err = nvs_get_i32(my_handle, "valveState2", &valveStateEEPROM); datashiftregister = 32; numShift = 0;break;
        case 2:err = nvs_get_i32(my_handle, "valveState3", &valveStateEEPROM); datashiftregister = 128; numShift = 1;break;
        case 3:err = nvs_get_i32(my_handle, "valveState4", &valveStateEEPROM); datashiftregister = 32; numShift = 1;break;
        case 4:err = nvs_get_i32(my_handle, "valveState5", &valveStateEEPROM); datashiftregister = 128; numShift = 2;break;
        case 5:err = nvs_get_i32(my_handle, "valveState6", &valveStateEEPROM); datashiftregister = 32; numShift = 2;break;
        case 6:err = nvs_get_i32(my_handle, "valveState7", &valveStateEEPROM); datashiftregister = 128; numShift = 3;break;
        case 7:err = nvs_get_i32(my_handle, "valveState8", &valveStateEEPROM); datashiftregister = 32; numShift = 3;break;
        case 8:err = nvs_get_i32(my_handle, "valveState9", &valveStateEEPROM); datashiftregister = 128; numShift = 4;break;
        case 9:err = nvs_get_i32(my_handle, "valveState10", &valveStateEEPROM); datashiftregister = 32; numShift = 4;break;
        case 10:err = nvs_get_i32(my_handle, "valveState11", &valveStateEEPROM); datashiftregister = 128; numShift = 5;break;
        case 11:err = nvs_get_i32(my_handle, "valveState12", &valveStateEEPROM); datashiftregister = 32; numShift = 5;break;
        default:break;
        */

        case 0:err = nvs_get_i32(my_handle, "valveState1", &valveStateEEPROM); datashiftregister = 16; numShift = 0;break;
        case 1:err = nvs_get_i32(my_handle, "valveState2", &valveStateEEPROM); datashiftregister = 64; numShift = 0;break;
        case 2:err = nvs_get_i32(my_handle, "valveState3", &valveStateEEPROM); datashiftregister = 16; numShift = 1;break;
        case 3:err = nvs_get_i32(my_handle, "valveState4", &valveStateEEPROM); datashiftregister = 64; numShift = 1;break;
        case 4:err = nvs_get_i32(my_handle, "valveState5", &valveStateEEPROM); datashiftregister = 16; numShift = 2;break;
        case 5:err = nvs_get_i32(my_handle, "valveState6", &valveStateEEPROM); datashiftregister = 64; numShift = 2;break;
        case 6:err = nvs_get_i32(my_handle, "valveState7", &valveStateEEPROM); datashiftregister = 16; numShift = 3;break;
        case 7:err = nvs_get_i32(my_handle, "valveState8", &valveStateEEPROM); datashiftregister = 64; numShift = 3;break;
        case 8:err = nvs_get_i32(my_handle, "valveState9", &valveStateEEPROM); datashiftregister = 16; numShift = 4;break;
        case 9:err = nvs_get_i32(my_handle, "valveState10", &valveStateEEPROM); datashiftregister = 64; numShift = 4;break;
        case 10:err = nvs_get_i32(my_handle, "valveState11", &valveStateEEPROM); datashiftregister = 16; numShift = 5;break;
        case 11:err = nvs_get_i32(my_handle, "valveState12", &valveStateEEPROM); datashiftregister = 64; numShift = 5;break;
        default:break;
        
        /*
        case 0:err = nvs_get_i32(my_handle, "valveState1", &valveStateEEPROM);gpio_right = GPIO_RIGHT_1; gpio_left = GPIO_LEFT_1;break;
        case 1:err = nvs_get_i32(my_handle, "valveState2", &valveStateEEPROM);gpio_right = GPIO_RIGHT_2; gpio_left = GPIO_LEFT_2;break;
        default:break;
        */
    }

    switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("Valve State variable = %d\n", valveStateEEPROM);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
    }
    //if(valveStateEEPROM == 100){
    if(valveStateEEPROM > 0 || setup == true){
        //attiva motore con datashiftregsiter e numshift
        shiftOut(DATA_GPIO, CLOCK_GPIO, 0, datashiftregister);
        for(int i = 0; i < numShift;i++){
            shiftOut(DATA_GPIO, CLOCK_GPIO, 0, 0);
        }
        
        gpio_set_level(LATCH_GPIO, 1);
        gpio_set_level(LATCH_GPIO, 0);
        
        if (setup == false){
                readValue_off(motor_n);
        }
        else if (setup == true){
                result = (int)timerSetting_OFF();
                //result = readValue_off(motor_n);
        }

        clearShiftRegisters();

        switch(motor_n){
            case 0: printf("Setting the value of valve 1 to 0\n");
                    err = nvs_set_i32(my_handle,"valveState1",0);
                    ValveStates[0] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 1: printf("Setting the valueof valve 2 to 0\n");
                    err = nvs_set_i32(my_handle,"valveState2",0);
                    ValveStates[1] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            
            case 2: printf("Setting the value of valve 3 to 0\n");
                    err = nvs_set_i32(my_handle,"valveState3",0);
                    ValveStates[2] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 3: printf("Setting the valueof valve 4 to 0\n");
                    err = nvs_set_i32(my_handle,"valveState4",0);
                    ValveStates[3] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 4: printf("Setting the value of valve 5 to 0\n");
                    err = nvs_set_i32(my_handle,"valveState5",0);
                    ValveStates[4] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 5: printf("Setting the valueof valve 6 to 0\n");
                    err = nvs_set_i32(my_handle,"valveState6",0);
                    ValveStates[5] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 6: printf("Setting the value of valve 7 to 0\n");
                    err = nvs_set_i32(my_handle,"valveState7",0);
                    ValveStates[6] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 7: printf("Setting the valueof valve 8 to 0\n");
                    err = nvs_set_i32(my_handle,"valveState8",0);
                    ValveStates[7] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 8: printf("Setting the value of valve 9 to 0\n");
                    err = nvs_set_i32(my_handle,"valveState9",0);
                    ValveStates[8] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 9: printf("Setting the valueof valve 10 to 0\n");
                    err = nvs_set_i32(my_handle,"valveState10",0);
                    ValveStates[9] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;  
            case 10: printf("Setting the value of valve 11 to 0\n");
                    err = nvs_set_i32(my_handle,"valveState11",0);
                    ValveStates[10] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 11: printf("Setting the valueof valve 12 to 0\n");
                    err = nvs_set_i32(my_handle,"valveState12",0);
                    ValveStates[11] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;              
            
            default:break;
        } 
    }
    nvs_close(my_handle);
    return result;
}
void settingMotors_1(){
    printf("Setting Motors New\n");
    int tmp_time = 0;
    for(int i = 0; i < 12; i++){
        
        printf("Opening Valve %i at startup\n",i);
        printf("Result At Startup: %i\n",OpeningValve(i,true,100));
        vTaskDelay(pdMS_TO_TICKS(100));

        //ClosingValve(i,true);
        //vTaskDelay(pdMS_TO_TICKS(100));
        //---------------------------------------------------------------------------------
        printf("Processing Average Time Closing of motor %i\n",i);
        average_time_closing[i] = ClosingValve(i,true);
        printf("Result: %i\n",average_time_closing[i]);
        vTaskDelay(pdMS_TO_TICKS(100));
        //---------------------------------------------------------------------------------
        printf("Processing Average Time Opening of motor %i\n",i);
        tmp_time = OpeningValve(i,true,100);
        if(tmp_time >= (max_time * 0.6) && tmp_time < (max_time * 1.4)){
            average_time_opening[i] = tmp_time;
        }else{
            average_time_opening[i] = max_time;
        }
        printf("Result: %i\n",average_time_opening[i]);
        vTaskDelay(pdMS_TO_TICKS(100));
        /*
        printf("Processing Average Time Closing of motor %i\n",i);
        average_time_closing[i] = ClosingValve(i,true);
        printf("Result: %i",average_time_closing[i]);
        vTaskDelay(pdMS_TO_TICKS(100));
        */
    }
    esp_err_t err;
    nvs_handle_t my_handle;

    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");
    }
    printf("Setting Configuration Motor to 1\n");
    motor_configured = 1;
    err = nvs_set_i32(my_handle,"motor",motor_configured);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Opening Time Motor 1\n");
    err = nvs_set_i32(my_handle,"opening1",average_time_opening[0]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Opening Time Motor 2\n");
    err = nvs_set_i32(my_handle,"opening2",average_time_opening[1]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Opening Time Motor 3\n");
    err = nvs_set_i32(my_handle,"opening3",average_time_opening[2]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Opening Time Motor 4\n");
    err = nvs_set_i32(my_handle,"opening4",average_time_opening[3]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Opening Time Motor 5\n");
    err = nvs_set_i32(my_handle,"opening5",average_time_opening[4]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Opening Time Motor 6\n");
    err = nvs_set_i32(my_handle,"opening6",average_time_opening[5]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Opening Time Motor 7\n");
    err = nvs_set_i32(my_handle,"opening7",average_time_opening[6]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Opening Time Motor 8\n");
    err = nvs_set_i32(my_handle,"opening8",average_time_opening[7]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Opening Time Motor 9\n");
    err = nvs_set_i32(my_handle,"opening9",average_time_opening[8]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Opening Time Motor 10\n");
    err = nvs_set_i32(my_handle,"opening10",average_time_opening[9]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Opening Time Motor 11\n");
    err = nvs_set_i32(my_handle,"opening11",average_time_opening[10]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Opening Time Motor 12\n");
    err = nvs_set_i32(my_handle,"opening12",average_time_opening[11]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Closing Time Motor 1\n");
    err = nvs_set_i32(my_handle,"closing1",average_time_closing[0]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Closing Time Motor 2\n");
    err = nvs_set_i32(my_handle,"closing2",average_time_closing[1]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Closing Time Motor 3\n");
    err = nvs_set_i32(my_handle,"closing3",average_time_closing[2]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Closing Time Motor 4\n");
    err = nvs_set_i32(my_handle,"closing4",average_time_closing[3]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Closing Time Motor 5\n");
    err = nvs_set_i32(my_handle,"closing5",average_time_closing[4]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Closing Time Motor 6\n");
    err = nvs_set_i32(my_handle,"closing6",average_time_closing[5]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Closing Time Motor 7\n");
    err = nvs_set_i32(my_handle,"closing7",average_time_closing[6]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Closing Time Motor 8\n");
    err = nvs_set_i32(my_handle,"closing8",average_time_closing[7]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Closing Time Motor 9\n");
    err = nvs_set_i32(my_handle,"closing9",average_time_closing[8]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");
            
    printf("Setting Average Closing Time Motor 10\n");
    err = nvs_set_i32(my_handle,"closing10",average_time_closing[9]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");
        
    printf("Setting Average Closing Time Motor 11\n");
    err = nvs_set_i32(my_handle,"closing11",average_time_closing[10]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    printf("Setting Average Closing Time Motor 12\n");
    err = nvs_set_i32(my_handle,"closing12",average_time_closing[11]);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");        
    printf("Committing updates in NVS ... ");

    err = nvs_commit(my_handle);
    nvs_close(my_handle);
    /*
    printf("Processing Average Time Opening\n");
    average_time_opening = OpeningValve(0,true,100);
    printf("Result: %i",average_time_opening);
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("Processing Average Time Closing\n");
    average_time_closing = ClosingValve(0,true);
    printf("Result: %i",average_time_closing);
    vTaskDelay(pdMS_TO_TICKS(500));
    */
}
/*
void settingMotors(){
    printf("Setting Motors\n");
    unsigned int tmp_ON = 0;
    unsigned int tmp_OFF = 0;
    for (int i = 0; i < 2; i ++){
        tmp_ON = 0;
        tmp_OFF = 0;
        for(int j = 0; j < 3; j++){
            tmp_ON = tmp_ON + (OpeningValve(i, true)/1000000);
            vTaskDelay(pdMS_TO_TICKS(1000));
            tmp_OFF = tmp_OFF + (ClosingValve(i, true)/1000000);
            vTaskDelay(pdMS_TO_TICKS(1000));       
        }
        tmp_ON = tmp_ON / 3;
        tmp_OFF = tmp_OFF / 3;
        averageONValues[i] = tmp_ON;
        averageOFFValues[i] = tmp_OFF;
        printf("Average Closing VALUE: %i\n",tmp_OFF);
        printf("Average Opening VALUE: %i\n",tmp_ON); 
    } 
}
*/
// START HTTPS CLIENT CODE +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void https_get_task(void *pvParameters)
{
    char REQUEST_1[512];
    cJSON *json_root = NULL;
    cJSON *json_mesh_id = NULL;
    cJSON *json_mesh_pass = NULL;
    cJSON *json_gateway_essid = NULL;
    cJSON *json_gateway_pass = NULL;
    cJSON *json_mqtt_url = NULL;
    cJSON *json_firmware_url = NULL;

    uint8_t sta_mac[6] = {0};
    char sta_mac_string[20] = {0};

    esp_efuse_mac_get_default(sta_mac);
    sprintf(sta_mac_string, "\"%x%x:%x%x:%x%x:%x%x:%x%x:%x%x\"", 
                    ((sta_mac[0]&240)>>4),(sta_mac[0]&15), ((sta_mac[1]&240)>>4),(sta_mac[1]&15), ((sta_mac[2]&240)>>4),(sta_mac[2]&15), ((sta_mac[3]&240)>>4),(sta_mac[3]&15), ((sta_mac[4]&240)>>4),(sta_mac[4]&15), ((sta_mac[5]&240)>>4),(sta_mac[5]&15));   
    printf("STA_MAC %s\n", sta_mac_string);
    sprintf(REQUEST_1, REQUEST, sta_mac_string);           
    puts(REQUEST_1);
    
    char buf[512];
    char json[512];
    int ret, len;

    while(1){
        esp_tls_cfg_t cfg = {
            .crt_bundle_attach = esp_crt_bundle_attach,
        };

        struct esp_tls *tls = esp_tls_conn_http_new(WEB_URL, &cfg);

        if(tls != NULL) {
            ESP_LOGI(TAG, "Connection established...");
        } else {
            ESP_LOGE(TAG, "Connection failed...");
            goto exit;
        }

        size_t written_bytes = 0;
        do {
            ret = esp_tls_conn_write(tls,
                                     REQUEST_1 + written_bytes,
                                     strlen(REQUEST_1) - written_bytes);
            if (ret >= 0) {
                ESP_LOGI(TAG, "%d bytes written", ret);
                written_bytes += ret;
            } else if (ret != ESP_TLS_ERR_SSL_WANT_READ  && ret != ESP_TLS_ERR_SSL_WANT_WRITE) {
                ESP_LOGE(TAG, "esp_tls_conn_write  returned 0x%x", ret);
                goto exit;
            }
        } while(written_bytes < strlen(REQUEST_1));

        ESP_LOGI(TAG, "Reading HTTP response...");
        
        //int64_t actualTime = esp_timer_get_time();

        do
        {
            len = sizeof(buf) - 1;
            bzero(buf, sizeof(buf));
            ret = esp_tls_conn_read(tls, (char *)buf, len);

            if(ret == ESP_TLS_ERR_SSL_WANT_WRITE  || ret == ESP_TLS_ERR_SSL_WANT_READ)
                continue;

            if(ret < 0)
            {
                ESP_LOGE(TAG, "esp_tls_conn_read  returned -0x%x", -ret);
                break;
            }

            if(ret == 0)
            {
                ESP_LOGI(TAG, "connection closed");
                break;
            }
    
            len = ret;
            ESP_LOGD(TAG, "%d bytes read", len);
            printf("RECEIVED:%s\n",buf);
                for(int i = 0; i < len; i++) {
                    if(buf[i] == '{'){
                        for(int j = 0; j < len - i; j++){
                            json[j] = buf[j + i];
                        }
                        json[len - i] = '\0';
                    }
                }
            printf("RECEIVED JSON:%s\n",json);
            json_root = cJSON_Parse(json);
            json_gateway_essid = cJSON_GetObjectItem(json_root,"gatewayESSID");
            json_gateway_pass = cJSON_GetObjectItem(json_root,"gatewayPassword");
            json_mesh_id = cJSON_GetObjectItem(json_root,"meshId");
            json_mesh_pass = cJSON_GetObjectItem(json_root,"meshPassword");
            json_mqtt_url = cJSON_GetObjectItem(json_root,"brokerURL");
            json_firmware_url = cJSON_GetObjectItem(json_root,"httpServerURL");
                
        } while(1);

    exit:
        esp_tls_conn_delete(tls);
        putchar('\n'); // JSON output doesn't have a newline at end
        if(json_gateway_essid == NULL || json_gateway_pass == NULL || json_mesh_id == NULL || 
            json_mesh_pass == NULL || json_mqtt_url == NULL || json_firmware_url == NULL){
            printf("Some parameter has not been initialized yet\n");
            if(configured == 1){
                break;
            }else{
                for(int countdown = 2; countdown >= 0; countdown--) {
                    ESP_LOGI(TAG, "%d...", countdown);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                ESP_LOGI(TAG, "Starting again!");
            }
        }

        else if(json_gateway_essid->valuestring == NULL || json_gateway_pass->valuestring == NULL || json_mesh_id->valuestring == NULL || 
            json_mesh_pass->valuestring == NULL || json_mqtt_url->valuestring == NULL || json_firmware_url->valuestring == NULL){
            printf("Some parameter has NULL values\n");
            if(configured == 1){
                break;
            }else{
                for(int countdown = 2; countdown >= 0; countdown--) {
                    ESP_LOGI(TAG, "%d...", countdown);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                ESP_LOGI(TAG, "Starting again!");
            }
        }

        else{
            printf("Configuration: \n");
            printf("Gateway ESSID: %s\n", json_gateway_essid->valuestring);
            printf("Gateway password: %s\n", json_gateway_pass->valuestring);
            printf("Mesh ID: %s\n", json_mesh_id->valuestring);
            printf("Mesh pass: %s\n", json_mesh_pass->valuestring);
            printf("MQTT URL: %s\n", json_mqtt_url->valuestring);
            printf("Firmware URL: %s\n", json_firmware_url->valuestring);
            printf("Update NVS variable and reboot\n");
            printf("Opening Non-Volatile Storage (NVS) handle... ");

            esp_err_t err;
            nvs_handle_t my_handle;
            err = nvs_open("storage", NVS_READWRITE, &my_handle);
            if (err != ESP_OK) {
                printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            } else {
                printf("Done\n");
            }
            printf("Setting the value to 1\n");
            int boot_start = 1;
            err = nvs_set_i32(my_handle,"boot_start",boot_start);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Updating Parameters\n");

            err = nvs_set_str(my_handle, "mesh_id", json_mesh_id->valuestring);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            err = nvs_set_str(my_handle, "mesh_password", json_mesh_pass->valuestring);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            err = nvs_set_str(my_handle, "Gateway_ESSID", json_gateway_essid->valuestring);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            err = nvs_set_str(my_handle, "MQTT_URL", json_mqtt_url->valuestring);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            err = nvs_set_str(my_handle, "Firmware_URL", json_firmware_url->valuestring);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            err = nvs_set_str(my_handle, "Gateway_Pass", json_gateway_pass->valuestring);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            nvs_close(my_handle);

            ESP_LOGI(TAG, "Restarting in 2 seconds!");
            for(int countdown = 2; countdown >= 0; countdown--) {
                ESP_LOGI(TAG, "%d...", countdown);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            fflush(stdout);
            esp_restart(); 
        }
    }

    //configured = 1;
    //main_routine();
    ESP_LOGI(TAG, "HTTP task is exit");
    vTaskDelete(NULL);
}

static void OTAUpdate(char *filePath, char *md5){
    printf("OTA Update of: %s with md5: %s\n",filePath, md5);
    mbedtls_md5_context ctx;
    unsigned char digest[MD5_MAX_LEN];
    mbedtls_md5_init(&ctx);
    mbedtls_md5_starts_ret(&ctx);
    mbedtls_md5_update_ret(&ctx, (unsigned const char*) filePath, strlen(filePath));
    mbedtls_md5_finish_ret(&ctx, digest);
    char digest_str[MD5_MAX_LEN * 2];
    for (int i = 0; i < MD5_MAX_LEN; i++) {
        sprintf(&digest_str[i * 2], "%02x", (unsigned int)digest[i]);
    }

    ESP_LOGI(TAG, "Computed MD5 hash of firmware name %s", digest_str);
    
    //1)confronto dell'md5 calcolato con quello presente nel messaggio mqtt
        
    printf("%s\n",md5);
    printf("%s\n",digest_str);
    if(strncmp(md5,digest_str,32) == 0){
        printf("MD5 Confirmed\n");
        esp_http_client_config_t ota_client_config = {
            //.url = filePath,
            .url = "http://192.168.5.1:9001/api/firmware/MKC",
            .skip_cert_common_name_check=true,
            .transport_type=HTTP_TRANSPORT_OVER_SSL,
            .cert_pem = server_cert_pem_start
        };
        esp_err_t ret = esp_https_ota(&ota_client_config);
        if (ret == ESP_OK) {
            printf("OTA OK, restarting...\n");
            esp_restart();
        } else {
            printf("OTA failed...\n");
        }
    }else{
        ESP_LOGI(TAG, "MD5 Mismatch");
    }
}

static void node_read (int channel, float value){
    char *sending_data = NULL;
    size_t size_data = 0;
    uint8_t sta_mac[6] = { 0 };
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    int value_h = floor(value);
    int value_l = 0;
    if(value >= 0){
        value_l = (value - value_h) * 10;
    }else{
        int value_int = value * 10;
        int a = (abs(value_h)) * 10;
        value_l = a - abs(value_int);
    }
    switch (channel){
        case 1:
			printf("MSG CODE: %i\n", channel);
            if(value_h > 0 && value_h <= 100){
                if(ValveStates[channel - 1] != value_h){
                    printf("OPENING VALVE\n");
                    ClosingValve(0, false); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(0, false,value_h);
                }
            }else if(value_h == 0){
                printf("CLOSING VALVE\n");
                ClosingValve(0, false);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11]);
            /*
            if(MQTT_CONNEECTED){
                esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data); 
		break;

        case 2:
        	printf("MSG CODE: %i\n", channel);
            if(value_h > 0 && value_h <= 100){
                if(ValveStates[channel - 1] != value_h){
                    printf("OPENING VALVE\n");
                    ClosingValve(1, false); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(1, false, value_h);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(1, false);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
			size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11]);
			/*
            if(MQTT_CONNEECTED){
                esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 3:
        	printf("MSG CODE: %i\n", channel);
            if(value_h > 0 && value_h <= 100){
            	if(ValveStates[channel - 1] != value_h){
                	printf("OPENING VALVE\n");
                    ClosingValve(2, false); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(2, false, value_h);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(2, false);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 4:
        	printf("MSG CODE: %i\n", channel);
            if(value_h > 0 && value_h <= 100){
            	if(ValveStates[channel - 1] != value_h){
                	printf("OPENING VALVE\n");
                    ClosingValve(3, false); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(3, false, value_h);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(3, false);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 5:
        	printf("MSG CODE: %i\n", channel);
            if(value_h > 0 && value_h <= 100){
            	if(ValveStates[channel - 1] != value_h){
                	printf("OPENING VALVE\n");
                    ClosingValve(4, false); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(4, false, value_h);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(4, false);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 6:
        	printf("MSG CODE: %i\n", channel);
            if(value_h > 0 && value_h <= 100){
            	if(ValveStates[channel - 1] != value_h){
                	printf("OPENING VALVE\n");
                    ClosingValve(5, false); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(5, false, value_h);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(5, false);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
            	ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 7:
        	printf("MSG CODE: %i\n", channel);
            if(value_h > 0 && value_h <= 100){
            	if(ValveStates[channel - 1] != value_h){
                	printf("OPENING VALVE\n");
                    ClosingValve(6, false); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(6, false, value_h);
                }
            }else if(value_h == 0){
            printf("CLOSING VALVE\n");
            ClosingValve(6, false);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 8:
        	printf("MSG CODE: %i\n", channel);
            if(value_h > 0 && value_h <= 100){
            	if(ValveStates[channel - 1] != value_h){
                	printf("OPENING VALVE\n");
                    ClosingValve(7, false); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(7, false, value_h);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(7, false);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 9:
        	printf("MSG CODE: %i\n", channel);
            if(value_h > 0 && value_h <= 100){
            	if(ValveStates[channel - 1] != value_h){
                	printf("OPENING VALVE\n");
                    ClosingValve(8, false); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(8, false, value_h);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(8, false);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 10:
        	printf("MSG CODE: %i\n", channel);
            if(value_h > 0 && value_h <= 100){
                if(ValveStates[channel - 1] != value_h){
                	printf("OPENING VALVE\n");
                    ClosingValve(9, false); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(9, false, value_h);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(9, false);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
            	ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 11:
        	printf("MSG CODE: %i\n", channel);
            if(value_h > 0 && value_h <= 100){
            	if(ValveStates[channel - 1] != value_h){
                	printf("OPENING VALVE\n");
                    ClosingValve(10, false); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(10, false, value_h);
                 }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
            	ClosingValve(10, false);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

		case 12:
			printf("MSG CODE: %i\n", channel);
			if(value_h > 0 && value_h <= 100){
				if(ValveStates[channel - 1] != value_h){
					printf("OPENING VALVE\n");
                    ClosingValve(11, false); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(11, false, value_h);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(11, false);                       
            }
                        
			//sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

		case 13:
        	size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11]);
			if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
            	ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            free(sending_data);
		break;

        case 14:
			settingMotors_1();
        break;

		case 15:
        	printf("MSG CODE: %i\n", channel);
            max_time = value_h * 20;
        break;
        
        case -2:
            printf("Opening Non-Volatile Storage (NVS) handle... ");
            esp_err_t err;
            nvs_handle_t my_handle;
            err = nvs_open("storage", NVS_READWRITE, &my_handle);
            if (err != ESP_OK) {
                printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            } else {
                printf("Done\n");
            }
            printf("Setting the value to 0\n");
            int boot_start = 0;
            err = nvs_set_i32(my_handle,"boot_start",boot_start);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            nvs_close(my_handle);

            ESP_LOGI(TAG, "Restarting in 2 seconds!");
            for(int countdown = 2; countdown >= 0; countdown--) {
                ESP_LOGI(TAG, "%d...", countdown);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            esp_restart(); 

        break;

        default:
        	printf("INVALID CODE");
        break; 
	}
}

static void mqtt_parse_data(const char *payload, const char *topic, size_t topic_size, size_t payload_size){
    uint8_t sta_mac[6] = {0};
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    char *data = NULL;
    asprintf(&data,"%02x%02x%02x%02x%02x%02x",MAC2STR(sta_mac));
    
    printf("TOPIC=%.*s\r\n", topic_size,topic);
    printf("DATA=%.*s\r\n", payload_size,payload);

    int channel;
    float value;
    char macString [13];
    char *str = malloc(payload_size + 1);
    memcpy(str, payload, payload_size);
    str[payload_size] = '\0';
    cJSON *obj = cJSON_Parse(str);
    free(str);

    cJSON *addr = cJSON_GetObjectItem(obj, "mac");
    if (addr == NULL) {
        ESP_LOGI(TAG,"ADDRESS IS NULL --> GOTO EXIT");
        goto _exit;
    }

    char * macAddressString = addr->valuestring;
    int j = 0;
    for(int i = 0; i < 12; i++){
        switch(i){
            case 2: j = j + 1; break;
            case 4: j = j + 1; break;
            case 6: j = j + 1; break;
            case 8: j = j + 1; break;
            case 10: j = j + 1; break;
            default: break;
        }
        macString[i] = *(macAddressString + i + j);
    }
    macString[12] = '\0';

    printf("Mac Address received: %s\n",macString);
    printf("Mac Address STA: %s\n",data);
    int result = strcmp(macString, data); 
    if (result == 0){
        cJSON *code = cJSON_GetObjectItem(obj, "channel");
        if (code == NULL) {
            ESP_LOGI(TAG,"CHANNEL IS NULL --> GOTO EXIT");
            goto _exit;
        }
        channel = code->valueint;
        if (channel == -3){
            cJSON *firmware = cJSON_GetObjectItem(obj, "firmware");
            cJSON *md5 = cJSON_GetObjectItem(obj, "md5");
            if (firmware == NULL) {
                ESP_LOGI(TAG,"FIRMWARE IS NULL --> GOTO EXIT");
                goto _exit;
            }
            if(md5 == NULL){
                ESP_LOGI(TAG,"MD5 IS NULL --> GOTO EXIT");
                goto _exit;
            }
            char * firmwarePath = firmware->valuestring;
            char * md5Value = md5->valuestring;
            OTAUpdate(firmwarePath,md5Value);
        }
        else{
            cJSON *value_json = cJSON_GetObjectItem(obj, "value");
            if (value_json == NULL){
                if(channel == 0 || channel == -2 || channel == 14){
                    value = 0;
                }else{
                    ESP_LOGI(TAG,"VALUE IS NULL --> GOTO EXIT");
                    goto _exit;
                }
            }else{
                value = atof(value_json->valuestring);
            }

            printf("Channel: %i\n",channel);
            printf("Value: %f\n",value);
            node_read(channel,value);
        }
    }
 
_exit:
    cJSON_Delete(obj);
    free(data);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        initialTime = esp_timer_get_time();
        MQTT_CONNEECTED=1;
        
        msg_id = esp_mqtt_client_subscribe(client, "mesh/toDevice", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        MQTT_CONNEECTED=0;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        initialTime = esp_timer_get_time();
        //printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        //printf("DATA=%.*s\r\n", event->data_len, event->data);
        mqtt_parse_data(event->data,event->topic,event->topic_len,event->data_len);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    ESP_LOGI(TAG, "STARTING MQTT");
    esp_mqtt_client_config_t mqttConfig = {
        //.uri = "mqtt://192.168.5.1:1883"
        .uri = MQTT_URL
    };
    
    client = esp_mqtt_client_init(&mqttConfig);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

static void mqtt_app_stop(void){
    ESP_LOGI(TAG, "STOPPING MQTT");
    //esp_mqtt_client_unregister_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler);
    esp_mqtt_client_stop(client);
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
            printf("WIFI STARTED\n");
            esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY && configured == 0) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to Connect to the Configuration Network: %i", s_retry_num );
        } 
        else if(configured == 1){
            ESP_LOGI(TAG, "Retry to Connect to the Production Network");
            mqtt_app_stop();
            esp_wifi_connect();
        }
        else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        if(configured == 1){
            mqtt_app_start();
        }
    }
}


void wifi_init_sta() 
{
    s_wifi_event_group = xEventGroupCreate(); 

    ESP_ERROR_CHECK(esp_netif_init()); 

    ESP_ERROR_CHECK(esp_event_loop_create_default()); 
    sta = esp_netif_create_default_wifi_sta(); 

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); 

    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id)); 
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip)); 

    if (configured == 0){
        ESP_LOGI(TAG, "Try to Connect to the Configuration Network");
        wifi_config_t wifi_config = {
            .sta = {
                .ssid = ESP_WIFI_SSID,
                .password = ESP_WIFI_PASS
            },
        };
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
        ESP_ERROR_CHECK(esp_wifi_start() ); 
    }

    else{
        ESP_LOGI(TAG, "Try to Connect to the Production Network");
        main_routine();
    }

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        if (configured == 0) {
            ESP_LOGI(TAG, "Connected to Configuration Network");
            xTaskCreate(&https_get_task, "https_get_task", 8192, NULL, 5, NULL);
        }
    } else if (bits & WIFI_FAIL_BIT) {
        if(configured == 0){
            ESP_LOGI(TAG, "Failed to Connect to Configuration Network");
            ESP_LOGI(TAG, "Go in sleep mode");
            esp_sleep_enable_timer_wakeup(60 * 1000000);
            esp_deep_sleep_start(); 
        }
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

// FINISH HTTPS CLIENT CODE +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void node_write_task(void *arg)
{
    size_t size = 0;
    char *data = NULL;
    uint8_t sta_mac[6] = {0};

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    
    for (;;) {
        
        size = asprintf(&data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11]);

        if(MQTT_CONNEECTED){
                esp_mqtt_client_publish(client, "mesh/toCloud", data, 0, 0, 0);
        }
        free(data);
        vTaskDelay(30000 / portTICK_RATE_MS);
    }
    ESP_LOGI(TAG,"Node write task is exit");
    vTaskDelete(NULL);
}

static void check_network(void *arg){
    for(;;){
        if(esp_timer_get_time() - initialTime >= 900000000){
            printf("OPENING ALL VALVES\n");
            for(int i = 0; i < 12; i++){
                if(ValveStates[i]!=100){
                    ClosingValve(i, false); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(i, false,100);
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
    ESP_LOGI(TAG,"Check Network task is exit");
    vTaskDelete(NULL);
}

void main_routine(){

	vTaskDelay(5000 / portTICK_RATE_MS);
	
	esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
    	printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
    	printf("Done\n");
    }

    size_t size_2;
    //printf("first nvs_get_blob \n");
    err = nvs_get_str(my_handle, "Gateway_ESSID", NULL, &size_2);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error: %s\n", esp_err_to_name(err));
    }
    char* value_2 = malloc(size_2);
    //printf("second nvs_get_blob \n");
    err = nvs_get_str(my_handle, "Gateway_ESSID", value_2, &size_2);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Gateway ESSID = %s\n", value_2);
        break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
        break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }
    size_t size_3;
    //printf("first nvs_get_blob \n");
    err = nvs_get_str(my_handle, "Gateway_Pass", NULL, &size_3);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error: %s\n", esp_err_to_name(err));
    }
    char* value_3 = malloc(size_3);
    //printf("second nvs_get_blob \n");
    err = nvs_get_str(my_handle, "Gateway_Pass", value_3, &size_3);
    switch (err) {
        case ESP_OK:
            printf("Done\n");   
            printf("Gateway PASSWORD = %s\n", value_3);
        break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
        break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    nvs_close(my_handle);
	check_efuse();
    //Configure ADC
    if (unit == ADC_UNIT_1) {
    	adc1_config_width(width);
        adc1_config_channel_atten(channel, atten);
    } else {
    	adc2_config_channel_atten((adc2_channel_t)channel, atten);
	}
	//Characterize ADC
	adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
	print_char_val_type(val_type);
    /*
 	gpio_set_level(GPIO_SENSE_OUT, 1);
	int voltage_26 = gpio_get_level(GPIO_SENSE_IN);
	if(voltage_26 == 1){
		printf("MOTOR DC IS PRESENT\n");
	}else{
		printf("MOTOR DC IS NOT PRESENT\n");
	}
	gpio_set_level(GPIO_SENSE_OUT, 0);     
	if(voltage_26 == 1){
    	settingMotors();
        	settingSensors();
	}
    */
    	//gpio_set_level(GPIO_ENABLE, 1);
    	//settingMotors_1();
	if(motor_configured == 0){
		settingMotors_1();
	}
    
    

    esp_wifi_disconnect();
    esp_wifi_stop();

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS
        },
    };

    strcpy((char *)wifi_config.sta.ssid, value_2);
    strcpy((char *)wifi_config.sta.password, value_3);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start()); 

    //configured = 1;
    
    free(value_2);
    free(value_3);

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    
    ESP_LOGI(TAG,"Update watchDog network");
    initialTime = esp_timer_get_time();
    
    xTaskCreate(node_write_task, "node_write_task", 4 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    xTaskCreate(check_network, "check_network", 4 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
}

void app_main()
{	

    gpio_pad_select_gpio(SRCLR);
    gpio_pad_select_gpio(LATCH_GPIO);
    gpio_pad_select_gpio(CLOCK_GPIO);
    gpio_pad_select_gpio(DATA_GPIO);
    gpio_pad_select_gpio(POWER_GPIO);
    
    gpio_set_direction(SRCLR, GPIO_MODE_OUTPUT);
    gpio_set_direction(LATCH_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(CLOCK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(DATA_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(POWER_GPIO, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(PRES);
    gpio_set_pull_mode(PRES, GPIO_PULLUP_ONLY);
    gpio_set_direction(PRES, GPIO_MODE_INPUT);

	gpio_set_level(POWER_GPIO, 1);
    gpio_set_level(LATCH_GPIO, 0);
    gpio_set_level(CLOCK_GPIO, 0);
    gpio_set_level(DATA_GPIO, 0);
    gpio_set_level(SRCLR, 1);

    clearShiftRegisters();
    uint8_t test = 1;
    int numberOfModules = 0;
    int level = 0;
    shiftOut(DATA_GPIO, CLOCK_GPIO, 0, test);
    gpio_set_level(LATCH_GPIO, 1);
    gpio_set_level(LATCH_GPIO, 0);
    vTaskDelay(1000 / portTICK_RATE_MS);
    level = gpio_get_level(PRES);
    printf("Number of Slave modules: %d\n",level);
    level = 0;
    for(int i = 1; i < 6; i++){
        printf("MODULE: %d\n",i);
        shiftOut(DATA_GPIO, CLOCK_GPIO, 0, 0);
        gpio_set_level(LATCH_GPIO, 1);
        gpio_set_level(LATCH_GPIO, 0);
        vTaskDelay(1000 / portTICK_RATE_MS);
        level = gpio_get_level(PRES);
        printf("LEVEL: %d\n",level);
        numberOfModules = numberOfModules + level;
    }

    clearShiftRegisters();

    printf("NUMBER OF LEVEL: %d\n",numberOfModules);
    global_number_modules = numberOfModules;

    /*
    gpio_pad_select_gpio(GPIO_ENABLE);
    gpio_set_direction(GPIO_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_ENABLE, 0);
    
    gpio_pad_select_gpio(GPIO_RIGHT_1);
    gpio_set_direction(GPIO_RIGHT_1, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_RIGHT_1, 0);

    gpio_pad_select_gpio(GPIO_LEFT_1);
    gpio_set_direction(GPIO_LEFT_1, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LEFT_1, 0);

    gpio_pad_select_gpio(GPIO_RIGHT_2);
    gpio_set_direction(GPIO_RIGHT_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_RIGHT_2, 0);

    gpio_pad_select_gpio(GPIO_LEFT_2);
    gpio_set_direction(GPIO_LEFT_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LEFT_2, 0);
    */
    

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    // Open
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");
    }
    // Read
    printf("Reading boot start script from NVS ... ");
    int32_t boot_start = 0; // value will default to 0, if not set yet in NVS
    int32_t motor = 0;
    int32_t valveEEPROM1 = 0;
    int32_t valveEEPROM2 = 0;
    int32_t valveEEPROM3 = 0;
    int32_t valveEEPROM4 = 0;
    int32_t valveEEPROM5 = 0;
    int32_t valveEEPROM6 = 0;
    int32_t valveEEPROM7 = 0;
    int32_t valveEEPROM8 = 0;
    int32_t valveEEPROM9 = 0;
    int32_t valveEEPROM10 = 0;
    int32_t valveEEPROM11 = 0;
    int32_t valveEEPROM12 = 0;
    int opening1 = 0;
    int opening2 = 0;
    int opening3 = 0;
    int opening4 = 0;
    int opening5 = 0;
    int opening6 = 0;
    int opening7 = 0;
    int opening8 = 0;
    int opening9 = 0;
    int opening10 = 0;
    int opening11 = 0;
    int opening12 = 0;
    int closing1 = 0;
    int closing2 = 0;
    int closing3 = 0;
    int closing4 = 0;
    int closing5 = 0;
    int closing6 = 0;
    int closing7 = 0;
    int closing8 = 0;
    int closing9 = 0;
    int closing10 = 0;
    int closing11 = 0;
    int closing12 = 0;

    err = nvs_get_i32(my_handle, "boot_start", &boot_start);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Boot Start Variable = %d\n", boot_start);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"boot_start",boot_start);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "motor", &motor);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Motor Configuration = %d\n", motor);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"motor",motor);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "opening1", &opening1);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Opening Time Motor 1 = %d\n", opening1);
            average_time_opening[0] = opening1;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"opening1",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "opening2", &opening2);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Opening Time Motor 2 = %d\n", opening2);
            average_time_opening[1] = opening2;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"opening2",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "opening3", &opening3);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Opening Time Motor 3 = %d\n", opening3);
            average_time_opening[2] = opening3;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"opening3",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "opening4", &opening4);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Opening Time Motor 4 = %d\n", opening4);
            average_time_opening[3] = opening4;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"opening4",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "opening5", &opening5);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Opening Time Motor 5 = %d\n", opening5);
            average_time_opening[4] = opening5;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"opening5",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "opening6", &opening6);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Opening Time Motor 6 = %d\n", opening6);
            average_time_opening[5] = opening6;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"opening6",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "opening7", &opening7);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Opening Time Motor 7 = %d\n", opening7);
            average_time_opening[6] = opening7;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"opening7",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "opening8", &opening8);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Opening Time Motor 8 = %d\n", opening8);
            average_time_opening[7] = opening8;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"opening8",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "opening9", &opening9);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Opening Time Motor 9 = %d\n", opening9);
            average_time_opening[8] = opening9;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"opening9",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "opening10", &opening10);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Opening Time Motor 10 = %d\n", opening10);
            average_time_opening[9] = opening10;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"opening10",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "opening11", &opening11);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Opening Time Motor 11 = %d\n", opening11);
            average_time_opening[10] = opening11;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"opening11",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "opening12", &opening12);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Opening Time Motor 12 = %d\n", opening12);
            average_time_opening[11] = opening12;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"opening12",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "closing1", &closing1);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Closing Time Motor 1 = %d\n", closing1);
            average_time_closing[0] = closing1;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"closing1",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "closing2", &closing2);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Closing Time Motor 2 = %d\n", closing2);
            average_time_closing[1] = closing2;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"closing2",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "closing3", &closing3);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Closing Time Motor 3 = %d\n", closing3);
            average_time_closing[2] = closing3;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"closing3",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "closing4", &closing4);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Closing Time Motor 4 = %d\n", closing4);
            average_time_closing[3] = closing4;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"closing4",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "closing5", &closing5);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Closing Time Motor 5 = %d\n", closing5);
            average_time_closing[4] = closing5;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"closing5",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "closing6", &closing6);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Closing Time Motor 6 = %d\n", closing6);
            average_time_closing[5] = closing6;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"closing6",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "closing7", &closing7);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Closing Time Motor 7 = %d\n", closing7);
            average_time_closing[6] = closing7;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"closing7",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "closing8", &closing8);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Closing Time Motor 8 = %d\n", closing8);
            average_time_closing[7] = closing8;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"closing8",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "closing9", &closing9);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Closing Time Motor 9 = %d\n", closing9);
            average_time_closing[8] = closing9;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"closing9",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "closing10", &closing10);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Closing Time Motor 10 = %d\n", closing10);
            average_time_closing[9] = closing10;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"closing10",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "closing11", &closing11);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Closing Time Motor 11 = %d\n", closing11);
            average_time_closing[10] = closing11;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"closing11",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "closing12", &closing12);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Average Closing Time Motor 12 = %d\n", closing12);
            average_time_closing[11] = closing12;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 350\n");
            err = nvs_set_i32(my_handle,"closing12",350);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState1", &valveEEPROM1);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM1);
            ValveStates[0] = valveEEPROM1;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState1",valveEEPROM1);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState2", &valveEEPROM2);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM2);
            ValveStates[1] = valveEEPROM2;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState2",valveEEPROM2);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState3", &valveEEPROM3);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM3);
            ValveStates[2] = valveEEPROM3;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState3",valveEEPROM3);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState4", &valveEEPROM4);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM4);
            ValveStates[3] = valveEEPROM4;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState4",valveEEPROM4);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState5", &valveEEPROM5);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM5);
            ValveStates[4] = valveEEPROM5;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState5",valveEEPROM5);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState6", &valveEEPROM6);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM6);
            ValveStates[5] = valveEEPROM6;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState6",valveEEPROM6);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState7", &valveEEPROM7);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM7);
            ValveStates[6] = valveEEPROM7;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState7",valveEEPROM7);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState8", &valveEEPROM8);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM8);
            ValveStates[7] = valveEEPROM8;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState8",valveEEPROM8);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState9", &valveEEPROM9);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM9);
            ValveStates[8] = valveEEPROM9;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState9",valveEEPROM9);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState10", &valveEEPROM10);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM10);
            ValveStates[9] = valveEEPROM10;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState10",valveEEPROM10);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState11", &valveEEPROM11);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM11);
            ValveStates[10] = valveEEPROM11;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState11",valveEEPROM11);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState12", &valveEEPROM12);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM12);
            ValveStates[11] = valveEEPROM12;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState12",valveEEPROM12);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    size_t size;
    err = nvs_get_str(my_handle, "Firmware_URL", NULL, &size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error: %s\n", esp_err_to_name(err));
    }
    char* value = malloc(size);
    err = nvs_get_str(my_handle, "Firmware_URL", value, &size);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Firmware URL= %s\n", value);
        break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            err = nvs_set_str(my_handle, "Firmware_URL", CONFIG_FIRMWARE_UPGRADE_URL);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
        break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        break;
    }
    free(value);
    nvs_close(my_handle);
    vTaskDelay(pdMS_TO_TICKS(5000));
    configured = boot_start;
    motor_configured = motor;
    wifi_init_sta();    
}

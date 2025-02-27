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
#include "freertos/queue.h"
#include "mqtt_client.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_sleep.h"

#define ESP_INTR_FLAG_DEFAULT 0

SemaphoreHandle_t xSemaphore = NULL;
bool led_status = false;

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

//definizione motori
/*
#define VALVOLA1 13
#define VALVOLA2 14
#define VALVOLA3 27
#define VALVOLA4 26
#define VALVOLA5 25
#define VALVOLA6 33
#define VALVOLA7 32
#define VALVOLA8 23
#define VALVOLA9 22
#define VALVOLA10 21
#define VALVOLA11 19
#define VALVOLA12 18
#define VALVOLA13 17
#define VALVOLA14 16
#define VALVOLA15 4
*/
#define VALVOLA1 15
#define VALVOLA2 32
#define VALVOLA3 33
#define VALVOLA4 25
#define VALVOLA5 26
#define VALVOLA6 27
#define VALVOLA7 14
#define VALVOLA8 13

#define VALVOLA9 23
#define VALVOLA10 22
#define VALVOLA11 21
#define VALVOLA12 19
#define VALVOLA13 18
#define VALVOLA14 17
#define VALVOLA15 16
#define VALVOLA16 4
//#define RELE_CALDAIA 4 

#define SYNC 39
#define CONS 36

int stato_caldaia_ram = 0;
int contatore_impulsi = 0;

uint32_t MQTT_CONNEECTED = 0;

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
esp_netif_t *sta;
esp_event_handler_instance_t instance_any_id;
esp_event_handler_instance_t instance_got_ip;
int32_t configured = 0;
int global_number_modules = 15;

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

static int64_t initialTime = 0;
static int32_t ValveStates [20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static int32_t TempValveStates[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


void IRAM_ATTR sync_isr_handler(void* arg) {
    
    xSemaphoreGiveFromISR(xSemaphore, NULL);
}

static int checkValveStates(void){
    int tmpResult = 0;
    for(int i = 0; i < 20; i++){
        tmpResult = ValveStates[i] + tmpResult;
    }
    return tmpResult;
}

static unsigned int OpeningValve(int motor_n, int value, bool boot){
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

    switch(motor_n){
 
        case 0:err = nvs_get_i32(my_handle, "valveState1", &valveStateEEPROM); break;
        case 1:err = nvs_get_i32(my_handle, "valveState2", &valveStateEEPROM); break;
        case 2:err = nvs_get_i32(my_handle, "valveState3", &valveStateEEPROM); break;
        case 3:err = nvs_get_i32(my_handle, "valveState4", &valveStateEEPROM); break;
        case 4:err = nvs_get_i32(my_handle, "valveState5", &valveStateEEPROM); break;
        case 5:err = nvs_get_i32(my_handle, "valveState6", &valveStateEEPROM); break;
        case 6:err = nvs_get_i32(my_handle, "valveState7", &valveStateEEPROM); break;
        case 7:err = nvs_get_i32(my_handle, "valveState8", &valveStateEEPROM); break;
        case 8:err = nvs_get_i32(my_handle, "valveState9", &valveStateEEPROM); break;
        case 9:err = nvs_get_i32(my_handle, "valveState10", &valveStateEEPROM); break;
        case 10:err = nvs_get_i32(my_handle, "valveState11", &valveStateEEPROM); break;
        case 11:err = nvs_get_i32(my_handle, "valveState12", &valveStateEEPROM); break;
        case 12:err = nvs_get_i32(my_handle, "valveState13", &valveStateEEPROM); break;
        case 13:err = nvs_get_i32(my_handle, "valveState14", &valveStateEEPROM); break;
        case 14:err = nvs_get_i32(my_handle, "valveState15", &valveStateEEPROM); break;
        case 15:err = nvs_get_i32(my_handle, "valveState16", &valveStateEEPROM); break;
        case 16:err = nvs_get_i32(my_handle, "valveState17", &valveStateEEPROM); break;
        case 17:err = nvs_get_i32(my_handle, "valveState18", &valveStateEEPROM); break;
        case 18:err = nvs_get_i32(my_handle, "valveState19", &valveStateEEPROM); break;
        case 19:err = nvs_get_i32(my_handle, "valveState20", &valveStateEEPROM); break;
        default:break;

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

    if(valveStateEEPROM != value){   
        //attiva motore con datashiftregsiter e numshift
        
        switch(motor_n){
            case 0: printf("Setting the value of valve 1 to: %i\n",value);
                    gpio_set_level(VALVOLA1, 100);
                    err = nvs_set_i32(my_handle,"valveState1",value);
                    ValveStates[0] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 1: printf("Setting the valueof valve 2 to: %i\n",value);
                    gpio_set_level(VALVOLA2, 100);
                    err = nvs_set_i32(my_handle,"valveState2",value);
                    ValveStates[1] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 2: printf("Setting the value of valve 3 to: %i\n",value);
                    gpio_set_level(VALVOLA3, 100);
                    err = nvs_set_i32(my_handle,"valveState3",value);
                    ValveStates[2] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 3: printf("Setting the valueof valve 4 to: %i\n",value);
                    gpio_set_level(VALVOLA4, 100);
                    err = nvs_set_i32(my_handle,"valveState4",value);
                    ValveStates[3] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 4: printf("Setting the value of valve 5 to: %i\n",value);
                    gpio_set_level(VALVOLA5, 100);
                    err = nvs_set_i32(my_handle,"valveState5",value);
                    ValveStates[4] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 5: printf("Setting the valueof valve 6 to: %i\n",value);
                    gpio_set_level(VALVOLA6, 100);
                    err = nvs_set_i32(my_handle,"valveState6",value);
                    ValveStates[5] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 6: printf("Setting the value of valve 7 to: %i\n",value);
                    gpio_set_level(VALVOLA7, 100);
                    err = nvs_set_i32(my_handle,"valveState7",value);
                    ValveStates[6] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 7: printf("Setting the valueof valve 8 to: %i\n",value);
                    gpio_set_level(VALVOLA8, 100);
                    err = nvs_set_i32(my_handle,"valveState8",value);
                    ValveStates[7] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 8: printf("Setting the value of valve 9 to: %i\n",value);
                    gpio_set_level(VALVOLA9, 100);
                    err = nvs_set_i32(my_handle,"valveState9",value);
                    ValveStates[8] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 9: printf("Setting the valueof valve 10 to: %i\n",value);
                    gpio_set_level(VALVOLA10, 100);
                    err = nvs_set_i32(my_handle,"valveState10",value);
                    ValveStates[9] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;  
            case 10: printf("Setting the value of valve 11 to: %i\n",value);
                    gpio_set_level(VALVOLA11, 100);
                    err = nvs_set_i32(my_handle,"valveState11",value);
                    ValveStates[10] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 11: printf("Setting the valueof valve 12 to: %i\n",value);
                    gpio_set_level(VALVOLA12, 100);
                    err = nvs_set_i32(my_handle,"valveState12",value);
                    ValveStates[11] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break; 
            case 12: printf("Setting the valueof valve 13 to: %i\n",value);
                    gpio_set_level(VALVOLA13, 100);
                    err = nvs_set_i32(my_handle,"valveState13",value);
                    ValveStates[12] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break; 
            case 13: printf("Setting the valueof valve 14 to: %i\n",value);
                    gpio_set_level(VALVOLA14, 100);
                    err = nvs_set_i32(my_handle,"valveState14",value);
                    ValveStates[13] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;   
            case 14: printf("Setting the valueof valve 15 to: %i\n",value);
                    gpio_set_level(VALVOLA15, 100);
                    err = nvs_set_i32(my_handle,"valveState15",value);
                    ValveStates[14] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;      
            case 15: printf("Setting the valueof valve 16 to: %i\n",value);
                    gpio_set_level(VALVOLA16, 100);
                    err = nvs_set_i32(my_handle,"valveState16",value);
                    ValveStates[15] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 16: printf("Setting the valueof valve 17 to: %i\n",value);
                    //gpio_set_level(VALVOLA17, 100);
                    err = nvs_set_i32(my_handle,"valveState17",value);
                    ValveStates[16] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 17: printf("Setting the valueof valve 18 to: %i\n",value);
                    //gpio_set_level(VALVOLA18, 100);
                    err = nvs_set_i32(my_handle,"valveState18",value);
                    ValveStates[17] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 18: printf("Setting the valueof valve 19 to: %i\n",value);
                    //gpio_set_level(VALVOLA19, 100);
                    err = nvs_set_i32(my_handle,"valveState19",value);
                    ValveStates[18] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 19: printf("Setting the valueof valve 20 to: %i\n",value);
                    //gpio_set_level(VALVOLA20, 100);
                    err = nvs_set_i32(my_handle,"valveState20",value);
                    ValveStates[19] = value;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            default:break;
        } 
    }
    /*
    if (checkValveStates() != 0 && stato_caldaia_ram == 0 && boot == false){
        gpio_set_level(RELE_CALDAIA, 100);
    }
    if(gpio_get_level(CONS) == 0){ 
        stato_caldaia_ram = 100;
    }
    */
    nvs_close(my_handle);
    return result;
}

static unsigned int ClosingValve(int motor_n){
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

    switch(motor_n){

        case 0:err = nvs_get_i32(my_handle, "valveState1", &valveStateEEPROM); break;
        case 1:err = nvs_get_i32(my_handle, "valveState2", &valveStateEEPROM); break;
        case 2:err = nvs_get_i32(my_handle, "valveState3", &valveStateEEPROM); break;
        case 3:err = nvs_get_i32(my_handle, "valveState4", &valveStateEEPROM); break;
        case 4:err = nvs_get_i32(my_handle, "valveState5", &valveStateEEPROM); break;
        case 5:err = nvs_get_i32(my_handle, "valveState6", &valveStateEEPROM); break;
        case 6:err = nvs_get_i32(my_handle, "valveState7", &valveStateEEPROM); break;
        case 7:err = nvs_get_i32(my_handle, "valveState8", &valveStateEEPROM); break;
        case 8:err = nvs_get_i32(my_handle, "valveState9", &valveStateEEPROM); break;
        case 9:err = nvs_get_i32(my_handle, "valveState10", &valveStateEEPROM); break;
        case 10:err = nvs_get_i32(my_handle, "valveState11", &valveStateEEPROM); break;
        case 11:err = nvs_get_i32(my_handle, "valveState12", &valveStateEEPROM); break;
        case 12:err = nvs_get_i32(my_handle, "valveState13", &valveStateEEPROM); break;
        case 13:err = nvs_get_i32(my_handle, "valveState14", &valveStateEEPROM); break;
        case 14:err = nvs_get_i32(my_handle, "valveState15", &valveStateEEPROM); break;
        case 15:err = nvs_get_i32(my_handle, "valveState16", &valveStateEEPROM); break;
        case 16:err = nvs_get_i32(my_handle, "valveState17", &valveStateEEPROM); break;
        case 17:err = nvs_get_i32(my_handle, "valveState18", &valveStateEEPROM); break;
        case 18:err = nvs_get_i32(my_handle, "valveState19", &valveStateEEPROM); break;
        case 19:err = nvs_get_i32(my_handle, "valveState20", &valveStateEEPROM); break;
        default:break;
        
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

    if(valveStateEEPROM > 0){
        //attiva motore con datashiftregsiter e numshift


        switch(motor_n){
            case 0: printf("Setting the value of valve 1 to 0\n");
                    gpio_set_level(VALVOLA1, 0);
                    err = nvs_set_i32(my_handle,"valveState1",0);
                    ValveStates[0] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 1: printf("Setting the valueof valve 2 to 0\n");
                    gpio_set_level(VALVOLA2, 0);
                    err = nvs_set_i32(my_handle,"valveState2",0);
                    ValveStates[1] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            
            case 2: printf("Setting the value of valve 3 to 0\n");
                    gpio_set_level(VALVOLA3, 0);
                    err = nvs_set_i32(my_handle,"valveState3",0);
                    ValveStates[2] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 3: printf("Setting the valueof valve 4 to 0\n");
                    gpio_set_level(VALVOLA4, 0);
                    err = nvs_set_i32(my_handle,"valveState4",0);
                    ValveStates[3] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 4: printf("Setting the value of valve 5 to 0\n");
                    gpio_set_level(VALVOLA5, 0);
                    err = nvs_set_i32(my_handle,"valveState5",0);
                    ValveStates[4] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 5: printf("Setting the valueof valve 6 to 0\n");
                    gpio_set_level(VALVOLA6, 0);
                    err = nvs_set_i32(my_handle,"valveState6",0);
                    ValveStates[5] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 6: printf("Setting the value of valve 7 to 0\n");
                    gpio_set_level(VALVOLA7, 0);
                    err = nvs_set_i32(my_handle,"valveState7",0);
                    ValveStates[6] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 7: printf("Setting the valueof valve 8 to 0\n");
                    gpio_set_level(VALVOLA8, 0);
                    err = nvs_set_i32(my_handle,"valveState8",0);
                    ValveStates[7] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 8: printf("Setting the value of valve 9 to 0\n");
                    gpio_set_level(VALVOLA9, 0);
                    err = nvs_set_i32(my_handle,"valveState9",0);
                    ValveStates[8] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 9: printf("Setting the valueof valve 10 to 0\n");
                    gpio_set_level(VALVOLA10, 0);
                    err = nvs_set_i32(my_handle,"valveState10",0);
                    ValveStates[9] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;  
            case 10: printf("Setting the value of valve 11 to 0\n");
                    gpio_set_level(VALVOLA11, 0);
                    err = nvs_set_i32(my_handle,"valveState11",0);
                    ValveStates[10] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 11: printf("Setting the valueof valve 12 to 0\n");
                    gpio_set_level(VALVOLA12, 0);
                    err = nvs_set_i32(my_handle,"valveState12",0);
                    ValveStates[11] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;       
            case 12: printf("Setting the value of valve 13 to 0\n");
                    gpio_set_level(VALVOLA13, 0);
                    err = nvs_set_i32(my_handle,"valveState13",0);
                    ValveStates[12] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 13: printf("Setting the valueof valve 14 to 0\n");
                    gpio_set_level(VALVOLA14, 0);
                    err = nvs_set_i32(my_handle,"valveState14",0);
                    ValveStates[13] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 14: printf("Setting the value of valve 15 to 0\n");
                    gpio_set_level(VALVOLA15, 0);
                    err = nvs_set_i32(my_handle,"valveState15",0);
                    ValveStates[14] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 15: printf("Setting the valueof valve 16 to 0\n");
                    gpio_set_level(VALVOLA16, 0);
                    err = nvs_set_i32(my_handle,"valveState16",0);
                    ValveStates[15] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 16: printf("Setting the value of valve 17 to 0\n");
                    err = nvs_set_i32(my_handle,"valveState17",0);
                    ValveStates[16] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 17: printf("Setting the valueof valve 18 to 0\n");
                    err = nvs_set_i32(my_handle,"valveState18",0);
                    ValveStates[17] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 18: printf("Setting the value of valve 19 to 0\n");
                    err = nvs_set_i32(my_handle,"valveState19",0);
                    ValveStates[18] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;
            case 19: printf("Setting the valueof valve 20 to 0\n");
                    err = nvs_set_i32(my_handle,"valveState20",0);
                    ValveStates[19] = 0;
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                    printf("Committing updates in NVS ... ");
                    err = nvs_commit(my_handle);
                    printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
                    break;         
            
            default:break;
        } 
        /*
        if (checkValveStates() == 0 && stato_caldaia_ram == 100){
            gpio_set_level(RELE_CALDAIA, 0);
        }
        if(gpio_get_level(CONS) == 1){ 
            stato_caldaia_ram = 0;
        }
        */
    }
    nvs_close(my_handle);
    return result;
}

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
            //.url = "http://192.168.5.1:9001/api/firmware/MKC_TRIAC",
            .url = "http://192.168.1.249:9001/api/firmware/MKC_TRIAC", //Gasperini, Cocchi, Schirò
            //.url = "http://192.168.1.19:9001/api/firmware/MKC_TRIAC", //Degl'Innocenti Casa
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
                    ClosingValve(0); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(0,value_h, false);
                }
            }else if(value_h == 0){
                printf("CLOSING VALVE\n");
                ClosingValve(0);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
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
                    ClosingValve(1); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(1, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(1);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
			size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
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
                    ClosingValve(2); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(2, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(2);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
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
                    ClosingValve(3); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(3, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(3);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
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
                    ClosingValve(4); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(4, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(4);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
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
                    ClosingValve(5); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(5, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(5);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
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
                    ClosingValve(6); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(6, value_h, false);
                }
            }else if(value_h == 0){
                printf("CLOSING VALVE\n");
                ClosingValve(6);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
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
                    ClosingValve(7); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(7, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(7);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
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
                    ClosingValve(8); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(8, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(8);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
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
                    ClosingValve(9); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(9, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(9);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
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
                    ClosingValve(10); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(10, value_h, false);
                 }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
            	ClosingValve(10);                       
            }
                        
            //sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
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
                    ClosingValve(11); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(11, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(11);                       
            }
                        
			//sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 13:
			printf("MSG CODE: %i\n", channel);
			if(value_h > 0 && value_h <= 100){
				if(ValveStates[channel - 1] != value_h){
					printf("OPENING VALVE\n");
                    ClosingValve(12); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(12, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(12);                       
            }
                        
			//sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 14:
			printf("MSG CODE: %i\n", channel);
			if(value_h > 0 && value_h <= 100){
				if(ValveStates[channel - 1] != value_h){
					printf("OPENING VALVE\n");
                    ClosingValve(13); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(13, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(13);                       
            }
                        
			//sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 15:
			printf("MSG CODE: %i\n", channel);
			if(value_h > 0 && value_h <= 100){
				if(ValveStates[channel - 1] != value_h){
					printf("OPENING VALVE\n");
                    ClosingValve(14); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(14, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(14);                       
            }
                        
			//sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 16:
			printf("MSG CODE: %i\n", channel);
			if(value_h > 0 && value_h <= 100){
				if(ValveStates[channel - 1] != value_h){
					printf("OPENING VALVE\n");
                    ClosingValve(15); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(15, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(15);                       
            }
                        
			//sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 17:
			printf("MSG CODE: %i\n", channel);
			if(value_h > 0 && value_h <= 100){
				if(ValveStates[channel - 1] != value_h){
					printf("OPENING VALVE\n");
                    ClosingValve(16); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(16, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(16);                       
            }
                        
			//sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 18:
			printf("MSG CODE: %i\n", channel);
			if(value_h > 0 && value_h <= 100){
				if(ValveStates[channel - 1] != value_h){
					printf("OPENING VALVE\n");
                    ClosingValve(17); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(17, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(17);                       
            }
                        
			//sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 19:
			printf("MSG CODE: %i\n", channel);
			if(value_h > 0 && value_h <= 100){
				if(ValveStates[channel - 1] != value_h){
					printf("OPENING VALVE\n");
                    ClosingValve(18); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(18, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(18);                       
            }
                        
			//sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

        case 20:
			printf("MSG CODE: %i\n", channel);
			if(value_h > 0 && value_h <= 100){
				if(ValveStates[channel - 1] != value_h){
					printf("OPENING VALVE\n");
                    ClosingValve(19); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(19, value_h, false);
                }
            }else if(value_h == 0){
            	printf("CLOSING VALVE\n");
                ClosingValve(19);                       
            }
                        
			//sprintf(valveState, "%d", ValveState);
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d,\"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
			/*
            if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            */
            free(sending_data);
		break;

		case 21:
        	size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d, \"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);
			if(MQTT_CONNEECTED){
            	esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
            	ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            free(sending_data);
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
        .uri = "mqtt://192.168.1.249:1883" // Gasperini, Cocchi, Schirò
        //.uri = "mqtt://192.168.1.19:1883" // Degl'Innocenti casa
        //.uri = MQTT_URL
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
        //ESP_LOGI(TAG, "GPIO LEVEL: %d\n",gpio_get_level(CONS));
        if(gpio_get_level(CONS) == 0){ 
            stato_caldaia_ram = 0;
        } else{
            stato_caldaia_ram = 100;
        }
        
        size = asprintf(&data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"channel\": %d, \"valve_n\": %d, \"status\": %d,\"VALVE_1\":\"%d\",\"VALVE_2\":\"%d\",\"VALVE_3\":\"%d\",\"VALVE_4\":\"%d\",\"VALVE_5\":\"%d\",\"VALVE_6\":\"%d\",\"VALVE_7\":\"%d\",\"VALVE_8\":\"%d\",\"VALVE_9\":\"%d\",\"VALVE_10\":\"%d\",\"VALVE_11\":\"%d\",\"VALVE_12\":\"%d\",\"VALVE_13\":\"%d\",\"VALVE_14\":\"%d\",\"VALVE_15\":\"%d\",\"VALVE_16\":\"%d\",\"VALVE_17\":\"%d\",\"VALVE_18\":\"%d\",\"VALVE_19\":\"%d\",\"VALVE_20\":\"%d\"}", MAC2STR(sta_mac), 0, global_number_modules, stato_caldaia_ram, ValveStates[0], ValveStates[1], ValveStates[2],ValveStates[3],ValveStates[4],ValveStates[5],ValveStates[6],ValveStates[7],ValveStates[8],ValveStates[9],ValveStates[10],ValveStates[11],ValveStates[12],ValveStates[13],ValveStates[14],ValveStates[15],ValveStates[16],ValveStates[17],ValveStates[18],ValveStates[19]);

        if(MQTT_CONNEECTED){
                esp_mqtt_client_publish(client, "mesh/toCloud", data, 0, 0, 0);
        }
        free(data);
        vTaskDelay(30000 / portTICK_RATE_MS);
    }
    ESP_LOGI(TAG,"Node write task is exit");
    vTaskDelete(NULL);
}

void sync_task(void* arg) {
  for(;;) {
    if(xSemaphoreTake(xSemaphore,portMAX_DELAY) == pdTRUE) {
        if(contatore_impulsi == 100){
            contatore_impulsi = 0;
        } else{
            contatore_impulsi++;
        }
        printf("Sync trigger! -> Check motor status\n");
        ESP_LOGI(TAG, "Impulsi: %d", contatore_impulsi);
    }
  }
}

/*
static void check_network(void *arg){
    for(;;){
        if(esp_timer_get_time() - initialTime >= 900000000){
            printf("OPENING ALL VALVES AND TURN OFF THE BOILER\n");
            if(stato_caldaia_ram == 100){
                gpio_set_level(RELE_CALDAIA, 0);
                if(gpio_get_level(CONS) == 1){ 
                    stato_caldaia_ram = 0;
                }
            }
            for(int i = 0; i < 20; i++){
                if(ValveStates[i]!=100){
                    ClosingValve(i); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    OpeningValve(i,100, true);
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
    ESP_LOGI(TAG,"Check Network task is exit");
    vTaskDelete(NULL);
}
*/

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

    esp_wifi_disconnect();
    esp_wifi_stop();

    wifi_config_t wifi_config = {
        .sta = {
            //.ssid = ESP_WIFI_SSID, //Default
            //.password = ESP_WIFI_PASS //Default
            //.ssid = "MKT WiFi Network", //Gasperini
            //.password = "MKTPassword" //Gasperini
            //.ssid = "ES", //Degl'Innocenti casa
            //.password = "lupetto era ilmiocane" //Degl'Innocenti casa
            //.ssid = "TIM-29802241", //Cocchi
            //.password = "FWHpoS35fNP7CnEnCO5Tw0ks" //Cocchi
            .ssid = "TIM-42563755", //Schirò
            .password = "ZKd4Nfb3sSbN3TfDy9XT4kFz" //Schirò
        },
    };

    //strcpy((char *)wifi_config.sta.ssid, value_2); //Default
    //strcpy((char *)wifi_config.sta.password, value_3); //Default

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
    //xTaskCreate(check_network, "check_network", 4 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);

    //**************************** SYNC INITIALIZATION START*************************
    /*
    // start the task that will handle the sync trigger
	xTaskCreate(sync_task, "sync_task", 2048, NULL, 10, NULL);
	
	// install ISR service with default configuration
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	
	// attach the interrupt service routine
	gpio_isr_handler_add(SYNC, sync_isr_handler, NULL);
    */

     //**************************** SYNC INITIALIZATION END*************************

    //in caso di riavvio improvviso verifico le valvole che erano aperte ed in caso affermativo le chiudo.
    
    for (int i = 0; i < 20; i++){
        if(ValveStates[i] != 0){
            int tmpValue = ValveStates[i];
            ClosingValve(i);
            OpeningValve(i,tmpValue,true);
        }
    }
    
    //in caso di riavvio improvviso apro tutte le valvole ma non accendo la caldaia
    /*
    for (int i = 0; i < 20; i++){
        ClosingValve(i);
        OpeningValve(i,100, true);
    }
    gpio_set_level(RELE_CALDAIA, 0);
    if(gpio_get_level(CONS) == 1){ 
        stato_caldaia_ram = 0;
    }
    */
    
}

void app_main()
{	
    xSemaphore = xSemaphoreCreateBinary();

    gpio_pad_select_gpio(VALVOLA1);
    gpio_pad_select_gpio(VALVOLA2);
    gpio_pad_select_gpio(VALVOLA3);
    gpio_pad_select_gpio(VALVOLA4);
    gpio_pad_select_gpio(VALVOLA5);
    gpio_pad_select_gpio(VALVOLA6);
    gpio_pad_select_gpio(VALVOLA7);
    gpio_pad_select_gpio(VALVOLA8);
    gpio_pad_select_gpio(VALVOLA9);
    gpio_pad_select_gpio(VALVOLA10);
    gpio_pad_select_gpio(VALVOLA11);
    gpio_pad_select_gpio(VALVOLA12);
    gpio_pad_select_gpio(VALVOLA13);
    gpio_pad_select_gpio(VALVOLA14);

    //gpio_pad_select_gpio(RELE_CALDAIA);

    gpio_pad_select_gpio(SYNC);
    gpio_pad_select_gpio(CONS);
    
    gpio_set_direction(VALVOLA1, GPIO_MODE_OUTPUT);
    gpio_set_direction(VALVOLA2, GPIO_MODE_OUTPUT);
    gpio_set_direction(VALVOLA3, GPIO_MODE_OUTPUT);
    gpio_set_direction(VALVOLA4, GPIO_MODE_OUTPUT);
    gpio_set_direction(VALVOLA5, GPIO_MODE_OUTPUT);
    gpio_set_direction(VALVOLA6, GPIO_MODE_OUTPUT);
    gpio_set_direction(VALVOLA7, GPIO_MODE_OUTPUT);
    gpio_set_direction(VALVOLA8, GPIO_MODE_OUTPUT);
    gpio_set_direction(VALVOLA9, GPIO_MODE_OUTPUT);
    gpio_set_direction(VALVOLA10, GPIO_MODE_OUTPUT);
    gpio_set_direction(VALVOLA11, GPIO_MODE_OUTPUT);
    gpio_set_direction(VALVOLA12, GPIO_MODE_OUTPUT);
    gpio_set_direction(VALVOLA13, GPIO_MODE_OUTPUT);
    gpio_set_direction(VALVOLA14, GPIO_MODE_OUTPUT);
    gpio_set_direction(VALVOLA15, GPIO_MODE_OUTPUT);
    gpio_set_direction(VALVOLA16, GPIO_MODE_OUTPUT);

    //gpio_set_direction(RELE_CALDAIA, GPIO_MODE_OUTPUT);
    
    gpio_set_pull_mode(SYNC, GPIO_PULLDOWN_ONLY);
    gpio_set_direction(SYNC, GPIO_MODE_INPUT);
    //gpio_set_intr_type(SYNC, GPIO_INTR_NEGEDGE);
    

    gpio_set_pull_mode(CONS, GPIO_PULLUP_ONLY);
    gpio_set_direction(CONS, GPIO_MODE_INPUT);

    gpio_set_level(VALVOLA1, 0);
    gpio_set_level(VALVOLA2, 0);
    gpio_set_level(VALVOLA3, 0);
    gpio_set_level(VALVOLA4, 0);
    gpio_set_level(VALVOLA5, 0);
    gpio_set_level(VALVOLA6, 0);
    gpio_set_level(VALVOLA7, 0);
    gpio_set_level(VALVOLA8, 0);
    gpio_set_level(VALVOLA9, 0);
    gpio_set_level(VALVOLA10, 0);
    gpio_set_level(VALVOLA11, 0);
    gpio_set_level(VALVOLA12, 0);
    gpio_set_level(VALVOLA13, 0);
    gpio_set_level(VALVOLA14, 0);
    gpio_set_level(VALVOLA15, 0);
    gpio_set_level(VALVOLA16, 0);

    //gpio_set_level(RELE_CALDAIA, 0);

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
    int32_t valveEEPROM13 = 0;
    int32_t valveEEPROM14 = 0;
    int32_t valveEEPROM15 = 0;
    int32_t valveEEPROM16 = 0;
    int32_t valveEEPROM17 = 0;
    int32_t valveEEPROM18 = 0;
    int32_t valveEEPROM19 = 0;
    int32_t valveEEPROM20 = 0;
    
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

    err = nvs_get_i32(my_handle, "valveState13", &valveEEPROM13);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM13);
            ValveStates[12] = valveEEPROM13;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState13",valveEEPROM13);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState14", &valveEEPROM14);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM14);
            ValveStates[13] = valveEEPROM14;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState14",valveEEPROM14);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState15", &valveEEPROM15);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM15);
            ValveStates[14] = valveEEPROM15;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState15",valveEEPROM15);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState16", &valveEEPROM16);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM16);
            ValveStates[15] = valveEEPROM16;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState16",valveEEPROM16);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState17", &valveEEPROM17);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM17);
            ValveStates[16] = valveEEPROM17;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState17",valveEEPROM17);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState18", &valveEEPROM18);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM18);
            ValveStates[17] = valveEEPROM18;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState18",valveEEPROM18);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState19", &valveEEPROM19);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM19);
            ValveStates[18] = valveEEPROM19;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState19",valveEEPROM19);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    err = nvs_get_i32(my_handle, "valveState20", &valveEEPROM20);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Valve State variable = %d\n", valveEEPROM20);
            ValveStates[19] = valveEEPROM20;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to 0\n");
            err = nvs_set_i32(my_handle,"valveState20",valveEEPROM20);
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
    wifi_init_sta();    
}

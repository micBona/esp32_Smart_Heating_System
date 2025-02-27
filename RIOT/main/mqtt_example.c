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

#include "mdf_common.h"
#include "mesh_mqtt_handle.h"
#include "mwifi.h"
#include "mupgrade.h"
#include "driver/uart.h"
#include <stdio.h>
#include <string.h>
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
#include "mlink.h"
#include <unistd.h>
#include "esp_timer.h"
#include "esp_sleep.h"
#include "freertos/semphr.h"
#include "esp_event_loop.h"

//#define GPIO_ROOT 16
#define RELE_PIN_ON 33
#define RELE_PIN_OFF 32
#define RELE_STATE 5
#define OT_IN_PIN 16
#define OT_OUT_PIN 17

#define MESSAGES_COUNT 54
#define MEMORY_DEBUG
#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WEB_SERVER         CONFIG_WEB_SERVER
#define WEB_URL            CONFIG_WEB_URL
#define WEB_PORT           CONFIG_WEB_PORT

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
esp_netif_t *sta_netif;
esp_netif_t *sta;
esp_event_handler_instance_t instance_any_id;
esp_event_handler_instance_t instance_got_ip;
int32_t configured = 0;


static const char *TAG = "RIOT";

static const char REQUEST[512] = "POST " WEB_URL " HTTP/1.0\r\n"
    "User-Agent: esp-idf/1.0 esp32\r\n"
    "Host: "WEB_SERVER"\r\n"
    "Content-Type: application/json\r\n"
    "Content-Length: 35\r\n"
    "\r\n"
    "{\"macAddress\": %s}";

static bool Event_ID_0 = false;
static bool Event_ID_1 = false;
static bool Event_ID_5 = false;
static bool Event_ID_25 = false;
static bool Event_ID_26 = false;
static bool Event_ID_56_R = false;
static bool Event_ID_57_R = false;
static bool Event_ID_56_W = false;
static bool Event_ID_57_W = false;
static uint8_t StatusFlag = 0;
static float Tset = 0;
static float TdhwSet = 0;
static float MaxTset = 0;
static bool old_boiler = false;
static bool openTherm = false;
static void AHT10Init();
static void AHT10_Init();
static void AHT10_RST();
static void AHT10_Mea();
static unsigned char AHT10_Status();
static unsigned char AHT10_CalEN();
static float * AHT10_Read_data();
static void main_task_mesh();

void esp_netif_destroy_default_wifi(void *esp_netif)
{
    if (esp_netif) {
        esp_wifi_clear_default_wifi_driver_and_handlers(esp_netif);
    }
    esp_netif_destroy(esp_netif);
}

void AHT10Init(){ //AHT10 Initialization Instruction

	// send the command
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (0x38 << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, 0xE1, true);
	i2c_master_write_byte(cmd, 0x08, true);
	i2c_master_write_byte(cmd, 0x00, true);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
}

void AHT10_Init(){ //Initialization

	vTaskDelay(50 / portTICK_RATE_MS);   
	AHT10Init();
    // printf("AHT10Init()\n");
    
	if(AHT10_CalEN() == 0){
        // printf("if AHT10_CalEN() == 0\n");        
		while(AHT10_CalEN()==0){
            // printf("while AHT10_CalEN()\n");            
			AHT10_RST();
            // printf("AHT10_RST()\n");
			vTaskDelay(25 / portTICK_RATE_MS);            
			if(AHT10_CalEN() == 1){
                // printf("if AHT10_CalEN() == 1\n");
				break;
			}
		}
	}
}

void AHT10_RST(){ //Reset
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (0x38 << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, 0xba, true);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
}

void AHT10_Mea(){ //Trigger Measurement
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (0x38 << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, 0xac, true);
	i2c_master_write_byte(cmd, 0x33, true);
	i2c_master_write_byte(cmd, 0x00, true);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
}

unsigned char AHT10_Status(){ //Read AHT10 status register
	unsigned char byte_first;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (0x38 << 1) | I2C_MASTER_READ, true);
	i2c_master_read_byte(cmd, &byte_first, 0x01);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return byte_first;
}

unsigned char AHT10_CalEN(){ //Judge AHT10 calibration enable
	unsigned char val;
	val = AHT10_Status();
	if((val & 0x68) == 0x08){
		return 1;
	}
	else return 0;
}

float * AHT10_Read_data(){
	unsigned char byte_1th = 0;
	unsigned char byte_2th = 0;
	unsigned char byte_3th = 0;
	unsigned char byte_4th = 0;
	unsigned char byte_5th = 0;
	unsigned char byte_6th = 0;
	unsigned long humidity = 0;
	unsigned long temperature = 0;
	float humidity_Final = 0;
	float temp_Final = 0;
    float * p = malloc(sizeof(float) * 2);
	unsigned char cnt = 0;
	AHT10_Mea();
	vTaskDelay(80 / portTICK_RATE_MS);
	cnt = 0;
	while(((AHT10_Status() & 0x80)==0x80))
	{
		vTaskDelay(3 / portTICK_RATE_MS);
		if(cnt++>=100){
			break;
		}
	}
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (0x38 << 1) | I2C_MASTER_READ, true);
	i2c_master_read_byte(cmd, &byte_1th, 0x00);
	i2c_master_read_byte(cmd, &byte_2th, 0x00);
	i2c_master_read_byte(cmd, &byte_3th, 0x00);
	i2c_master_read_byte(cmd, &byte_4th, 0x00);
	i2c_master_read_byte(cmd, &byte_5th, 0x00);
	i2c_master_read_byte(cmd, &byte_6th, 0x01);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
    /*
	printf("STATE:%d\n", byte_1th);
	printf("HUMIDITY MSB: %d\n",byte_2th);
	printf("HUMIDITY LSB: %d\n",byte_3th);
	printf("HUM_TEMP MSB: %d\n",byte_4th);
	printf("TEMP LSB: %d\n",byte_5th);
	printf("TEMP LSB: %d\n",byte_6th);
    */
	humidity = 0;
	humidity = (byte_2th<<8)|byte_3th;
	humidity = ((humidity<<8)|byte_4th)>>4;
	humidity = humidity & 0x000fffff;
	
	humidity_Final = (float) (((float) humidity/1048576) * 100);

	temperature = 0;
	temperature = ((byte_4th % 16)<<8)|byte_5th;
	temperature = (temperature<<8)|byte_6th;
	temperature = temperature & 0x000fffff;
	temp_Final = (float) ((float) ((float)temperature/1048576) * 200) - 50;

	// printf("HUMIDITY: %f\n",humidity_Final);
	// printf("TEMPERATURE: %f\n",temp_Final);

    p[0] = humidity_Final;
    p[1] = temp_Final;
    return p;
}

void AHT10_initialization() {
    //printf("i2c scanner\r\n\r\n");
	// configure the i2c controller 0 in master mode, fast speed
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = 23;
	conf.scl_io_num = 22;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	printf("- i2c controller configured\r\n");
	
	// install the driver
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
	printf("- i2c driver installed\r\n\r\n");
	
	// printf("scanning the bus...\r\n\r\n");

	 int devices_found = 0;
	
	 for(int address = 1; address < 127; address++) {
	
	// 	// create and execute the command link
	 	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	 	i2c_master_start(cmd);
	 	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
	 	i2c_master_stop(cmd);
	 	if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS) == ESP_OK) {
	 		printf("-> found device with address 0x%02x\r\n", address);
			printf("Address Integer: %d\n", address);
			devices_found++;
		}
		i2c_cmd_link_delete(cmd);
	}
	if(devices_found == 0) printf("\r\n-> no devices found\r\n");
	// printf("\r\n...scan completed!\r\n");
}

static void checkResponse(unsigned long response);

enum DataType
{
  DT_FLAG8 = 0, // byte composed of 8 single-bit flags
  DT_U8 = 1, // unsigned 8-bit integer 0 .. 255
  DT_S8 = 2, // signed 8-bit integer -128 .. 127 (two’s compliment)
  DT_F88 = 3, // signed fixed point value : 1 sign bit, 7 integer bit, 
          // 8 fractional bits (two’s compliment ie. 
          // the LSB of the 16bit binary number represents 1/256th of a unit).
  DT_U16 = 4, // unsigned 16-bit integer 0..65535
  DT_S16 = 5, // signed 16-bit integer -32768..32767
  NONE = 6
};

struct OTMessage
{
  uint8_t dataId;
  char dataObject[32];
  uint8_t dataType_MSB;
  uint8_t dataType_LSB;
};

typedef enum  {ReadData = 0, WriteData = 1, InvalidData = 2, Reserved = 3} msgTypeM2S;
enum msgTypeS2M {ReadAck = 4, WriteAck = 5, DataInvalid = 6, Unknown_DataId = 7};

int spare = 0;
struct OTMessage messages[MESSAGES_COUNT] = {{0,"Status",DT_FLAG8,DT_FLAG8}, 
                {1,"Tset",DT_F88,NONE}, 
                {2,"Mconfig",DT_FLAG8,DT_U8}, 
                {3,"Sconfig",DT_FLAG8, DT_U8}, 
                {4,"Command",DT_U8, DT_U8}, 
                {5,"ASF_flags",DT_FLAG8, DT_U8}, 
                {6,"RBP_flags", DT_FLAG8, DT_FLAG8}, 
                {7, "CoolingControl", DT_F88, NONE}, 
                {8, "TsetCH2", DT_F88, NONE}, 
                {9, "TrOverride", DT_F88, NONE}, 
                {10, "TSP", DT_U8, DT_U8}, 
                {11, "TSP_Index", DT_U8, DT_U8}, 
                {12, "FHB_Size",DT_U8, DT_U8},
                {13, "FHB_Index", DT_U8, DT_U8}, 
                {14, "Max_rel_mod_level_setting",DT_F88, NONE}, 
                {15, "Max_Capacity_Min_Mod_Level",DT_U8, DT_U8}, 
                {16, "TrSet", DT_F88, NONE},
                {17, "Rel_mod_level",DT_F88, NONE}, 
                {18, "CH_pressure",DT_F88, NONE}, 
                {19, "DHW_flow_rate",DT_F88, NONE}, 
                {20, "Day_Time",DT_U8, NONE}, 
                {21, "Date", DT_U8, DT_U8}, 
                {22, "Year",DT_U16, NONE},
                {23, "TrSetCH2",DT_F88, NONE}, 
                {24, "Tr",DT_F88, NONE}, 
                {25, "Tboiler",DT_F88, NONE}, 
                {26, "Tdhw",DT_F88, NONE}, 
                {27, "Toutside",DT_F88, NONE}, 
                {28, "Tret",DT_F88, NONE}, 
                {29, "Tstorage",DT_F88, NONE}, 
                {30, "Tcollector",DT_F88, NONE}, 
                {31, "TflowCH2",DT_F88, NONE}, 
                {32, "Tdhw2",DT_F88, NONE}, 
                {33, "Texhaust",DT_S16, NONE}, 
                {48, "TdhwSet_UB_TdhwSet_LB",DT_S8, DT_S8},
                {49, "MaxTSet_UB_MaxTSet_LB",DT_S8,DT_S8}, 
                {50, "Hcratio_UB_Hcratio_LB",DT_S8,DT_S8}, 
                {56, "TdhwSet",DT_F88, NONE}, 
                {57, "MaxTSet",DT_F88,NONE}, 
                {58, "Hcratio",DT_F88,NONE}, 
                {100, "Remote_override_function",DT_FLAG8, NONE}, 
                {115, "OEM_diagnostic_code",DT_U16,NONE},
                {116, "Burner_starts",DT_U16, NONE}, 
                {117, "CH_pump_starts",DT_U16, NONE}, 
                {118, "DHW_pump_valve_starts",DT_U16, NONE}, 
                {119, "DHW_burner_starts",DT_U16, NONE},
                {120, "Burner_operation_hours",DT_U16, NONE}, 
                {121, "CH_pump_operation_hour",DT_U16, NONE}, 
                {122, "DHW_pump_valve_operation_hours",DT_U16, NONE},
                {123, "DHW_burner_operation_hours",DT_U16, NONE}, 
                {124, "OpenTherm_version_Master",DT_F88, NONE}, 
                {125, "OpenTherm_version_Slave",DT_F88, NONE},
                {126, "Master_version",DT_U8,DT_U8}, 
                {127, "Slave_version",DT_U8,DT_U8}};

unsigned long requests[] = {
    0x300, //0 get status
    0x90014000, //1 set CH temp
    0x80190000, //25 Boiler water temperature
};

uint16_t toF88(float f){
    uint16_t result = 0;
    if(f >= 0.0f){
        result = f * 256;
    }else{
        result = 0x10000 + f * 256;
    }
    return result;
}

float fromF88(uint16_t b){
    if(b & 0x8000){
        return (float) (-1.0 * (0x10000 - b) / 256.0f);
    }else{
        return (float)(b / 256.0f);
    }
}

void setIdleState(){
    gpio_set_level(OT_OUT_PIN, 1);
}

void setActiveState(){
    gpio_set_level(OT_OUT_PIN, 0);
}

void microsecondsDelay(int microseconds){
    int delta = 0;
    uint64_t tmpTime = esp_timer_get_time();
    while(delta < microseconds){
        delta = esp_timer_get_time() - tmpTime;
    }
    //printf("DELTA VALUE: %d\n", delta);
}

void activateBoiler(){
    printf("WAIT 10 SECONDS\n");
    vTaskDelay(10000);
    setIdleState();
    vTaskDelay(1000 /portTICK_PERIOD_MS);
}

void sendBit(bool high) {
    if (high) setActiveState(); else setIdleState();
    microsecondsDelay(500);
    if (high) setIdleState(); else setActiveState();
    microsecondsDelay(500);
}

bool bitRead(int number,int pos){
    return ((number >> pos) & 1);
}

void sendFrame(unsigned long request) {
    sendBit(true); //start bit
    for (int i = 31; i >= 0; i--){
        sendBit(bitRead(request, i));
    }
    sendBit(true); //stop bit
    setIdleState();
}

void printBinary(unsigned long val) {
    for (int i = 31; i >= 0; i--) {
        printf("%d",bitRead(val,i));
    }
}

bool waitForResponse() {
  int64_t time_stamp = esp_timer_get_time();
  while (gpio_get_level(OT_IN_PIN) != 1) { //start bit
    if (esp_timer_get_time() - time_stamp >= 800000) { //wait for response until 800 ms
      //printf("Response timeout\n");
      return false;
    }
  }
  //vTaskDelay(1.25 / portTICK_PERIOD_MS); //wait for first bit
  microsecondsDelay(1250);
  return true;
}

bool hasEvenParity(unsigned long request){ 
    int count = 0;
    for (int i = 0; i < 31; i++){
        if(((request>>i)&1) == 1){
            count++;
        }
    }
    if ((count%2) == 0){ // This means that the 32th bit should be 0
        return true;
    } else return false; // This means that the 32th bit should be 1
}

unsigned long readResponse_1(unsigned long response) {
    // printf("Response:  ");
    // printBinary(response);
    // printf("  /  ");
    // printf("%#010x\n", (int) response);
    // if ((response >> 16 & 0xFF) == 25) {
    //     printf("t=");
    //     printf("%lu", response >> 8 & 0xFF);
    //     printf("\n");
    // }
    if((hasEvenParity(response) == true) && (((response>>31) & 1) == 0)){
        printf("Check EVEN OK\n");
    }else if((hasEvenParity(response) == false) && (((response>>31) & 1) == 1)){
        printf("Check OVEN OK\n");
    }else{
        printf("Check Parity FAILED\n");
    }
    checkResponse(response);
    return response;
}


unsigned long readResponse() {
    unsigned long response = 0;
    for (int i = 0; i < 32; i++) {
        response = (response << 1) | gpio_get_level(OT_IN_PIN) ;
        microsecondsDelay(1000);
    }
    // printf("Response:  ");
    // printBinary(response);
    // printf("  /  ");
    // printf("%#010x\n", (int) response);
    // if ((response >> 16 & 0xFF) == 25) {
    //     printf("t=");
    //     printf("%lu", response >> 8 & 0xFF);
    //     printf("\n");
    // }
    if((hasEvenParity(response) == true) && (((response>>31) & 1) == 0)){
        printf("Check Parity OK\n");
    }else if((hasEvenParity(response) == false) && (((response>>31) & 1) == 1)){
        printf("Check Parity OK\n");
    }else{
        printf("Check Parity FAILED\n");
    }
    //checkResponse(response);
    return response;
}

unsigned long sendRequest(unsigned long request){
    // printf("Request:  ");
    // printBinary(request);
    // printf(" / ");
    // printf("%#010x\n", (int) request);
    sendFrame(request);
    microsecondsDelay(20000); //wait 20 mS before response
    if (!waitForResponse()) return 0;
    return readResponse();
}

uint16_t getDataValueU16(unsigned long response){
    return ((response & 255) | (response & (255 << 8)));
}

uint8_t getDataValueLSBU8(unsigned long response){
    uint8_t result = (response & 255); 
    printf("DATA VALUE: %u\n",result);
    return result;
}

uint8_t getDataValueMSBU8(unsigned long response){
    uint8_t result = ((response>>8) & 255);
    printf("DATA VALUE: %u\n",result);
    return result;
}

int16_t getDataValueS16(unsigned long response){
    int16_t result = ((response & 255) | (response & (255 << 8)));
    printf("DATA VALUE: %d\n",result);
    return result;
    
}

int8_t getDataValueLSBS8(unsigned long response){
    int8_t result = (response & 255);
    printf("DATA VALUE: %d\n",result);
    return result;
}

int8_t getDataValueMSBS8(unsigned long response){
    uint8_t result = ((response>>8) & 255);
    printf("DATA VALUE: %d\n",result);
    return result;
}

uint8_t getDataId(unsigned long response){
    return ((response>>16) & 255);
}

uint8_t getSpare(unsigned long response){
    return ((response>>24) & 15);
}

uint8_t getMsgType(unsigned long response){
    return ((response>>28) & 7);

}

unsigned long buildRequest(msgTypeM2S msgType, uint8_t dataId, uint8_t dataValueMSB, uint8_t dataValueLSB){
    unsigned long request = 0;
    request = (request | ( msgType << 28));
    request = request | (dataId << 16);
    request = request | (dataValueMSB << 8);
    request = request | dataValueLSB;
    return request;
}

unsigned long insertParity(unsigned long request){
    int count = 0;
    for (int i = 0; i < 31; i++){
        if(((request>>i)&1) == 1){
            count++;
        }
    }
    if ((count%2) != 0){ // if oven the 32th bit is set to 1 else to 0, example: 0000000000000000000000000000001 -> 10000000000000000000000000000001
        request = request | (1 << 31);
    }
    return request;
}
uint8_t createFlagByte(uint8_t MSB,uint8_t bit1,uint8_t bit2,uint8_t bit3,uint8_t bit4,uint8_t bit5,uint8_t bit6,uint8_t LSB){
    uint8_t flag = 0;
    flag = (flag | (MSB<<7) | (bit1<<6) | (bit2<<5) | (bit3<<4) | (bit4<<3) | (bit5<<2) | (bit6<<1) | (LSB));
    return flag;
}

void checkResponse(unsigned long response){
    uint8_t DataId = 0;
    uint8_t spareR = 0;
    uint8_t msgType = 0;
    float f88 = 0;
    uint16_t DataValueU16 = 0;
    uint8_t DataValueU8 = 0;
    DataId = getDataId(response);
    spareR = getSpare(response);
    msgType = getMsgType(response);
    printf("DATA ID: %u\n", DataId);
    printf("SPARE: %u\n", spareR);
    printf("MSG TYPE: %u\n", msgType);
    for (int i = 0; i < MESSAGES_COUNT; i++){
        if (messages[i].dataId == DataId){
            printf("DATA VALUE MSB\n");
            switch (messages[i].dataType_MSB){
                case 0: printf("DATA TYPE: DT_FLAG8\n");
                        DataValueU8 = getDataValueMSBU8(response);
                        printf("FLAG: ");
                        for (int i = 7; i >= 0; i--) {
                            printf("%d",bitRead(DataValueU8,i));
                        }
                        printf("\n");
                        break;
                case 1: printf("DATA TYPE: DT_U8\n");
                        getDataValueMSBU8(response);
                        break;
                case 2: printf("DATA TYPE: DT_S8\n");
                        getDataValueMSBS8(response);
                        break;
                case 3: printf("DATA TYPE: DT_F88\n");
                        f88 = fromF88(getDataValueU16(response));
                        printf("DATA VALUE: %f\n",f88);
                        break;
                case 4: printf("DATA TYPE: DT_U16\n");
                        DataValueU16 = getDataValueU16(response);
                        printf("DATA VALUE: %u\n",DataValueU16);
                        break;
                case 5: printf("DATA TYPE: DT_S16\n");
                        getDataValueS16(response);
                        break;
                case 6: printf("Nothing to do\n");
                        break;
                default: printf("UNKNOWN\n");
            }
            printf("DATA VALUE LSB\n");
            switch (messages[i].dataType_LSB){
                case 0: printf("DATA TYPE: DT_FLAG8\n");
                        DataValueU8 = getDataValueLSBU8(response);
                        printf("FLAG: ");
                        for (int i = 7; i >= 0; i--) { 
                            printf("%d",bitRead(DataValueU8,i));
                        }
                        printf("\n");
                        break;
                case 1: printf("DATA TYPE: DT_U8\n");
                        getDataValueLSBU8(response);
                        break;
                case 2: printf("DATA TYPE: DT_S8\n");
                        getDataValueLSBS8(response);
                        break;
                case 3: printf("DATA TYPE: DT_F88\n");
                        f88 = fromF88(getDataValueU16(response));
                        printf("DATA VALUE: %f\n",f88);
                        break;
                case 4: printf("DATA TYPE: DT_U16\n");
                        DataValueU16 = getDataValueU16(response);
                        printf("DATA VALUE: %u\n",DataValueU16);
                        break;
                case 5: printf("DATA TYPE: DT_S16\n");
                        getDataValueS16(response);
                        break;
                case 6: printf("Nothing to do\n");
                        break;
                default: printf("UNKNOWN\n");
            }
        }
    }
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

    uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};
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
        
        int64_t actualTime = esp_timer_get_time();

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
            // printf("Gateway ESSID: %s\n", json_gateway_essid->valuestring);
            // printf("Gateway password: %s\n", json_gateway_pass->valuestring);
            // printf("Mesh ID: %s\n", json_mesh_id->valuestring);
            // printf("Mesh pass: %s\n", json_mesh_pass->valuestring);
            // printf("MQTT URL: %s\n", json_mqtt_url->valuestring);
            // printf("Firmware URL: %s\n", json_firmware_url->valuestring);
                
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
    
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id);
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip);
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(sta));
    esp_netif_destroy(sta);
    esp_netif_deinit();
    vEventGroupDelete(s_wifi_event_group);
    main_task_mesh();
    MDF_LOGW("Node task is exit");
    vTaskDelete(NULL);
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
            printf("CONNECT!!!!");
            esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY && configured == 0) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta() //return boolean
{
    s_wifi_event_group = xEventGroupCreate(); //vEventGroupDelete(s_wifi_event_group);

    ESP_ERROR_CHECK(esp_netif_init()); //esp_netif_deinit();

    ESP_ERROR_CHECK(esp_event_loop_create_default()); //esp_event_loop_delete_default();
    sta = esp_netif_create_default_wifi_sta(); //esp_netif_destroy_default_wifi();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); //esp_wifi_deinit();

    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id)); //esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip)); //esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() ); //esp_wifi_stop();

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
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
                 xTaskCreate(&https_get_task, "https_get_task", 8192, NULL, 5, NULL);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
        if(configured == 1){
        ESP_LOGI(TAG, "Connect to Production Network");
            esp_wifi_disconnect();
            esp_wifi_stop();
            esp_wifi_deinit();
            esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id);
            esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip);
            ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(sta));
            esp_netif_destroy(sta);
            esp_netif_deinit();
            vEventGroupDelete(s_wifi_event_group);
            main_task_mesh();
        }
        else if(configured == 0){
            ESP_LOGI(TAG, "Go in sleep mode");
            esp_deep_sleep_start();
        }
        else{
            ESP_LOGI(TAG, "WRONG START BOOT");
        }
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    //ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    //ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    //vEventGroupDelete(s_wifi_event_group);
}

// FINISH HTTPS CLIENT CODE +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


static void root_write_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    //char *data = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    char *data = NULL;
    size_t size = MWIFI_PAYLOAD_LEN;
    uint8_t src_addr[MWIFI_ADDR_LEN] = { 0 };
    mwifi_data_type_t data_type = { 0 };

    MDF_LOGI("Root write task is running");

    while (esp_mesh_is_root()) {
        //size = MWIFI_PAYLOAD_LEN;
        //memset(data, 0, MWIFI_PAYLOAD_LEN);
        if (!mesh_mqtt_is_connect()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        /**
         * @brief Recv data from node, and forward to mqtt server.
         */
        //ret = mwifi_root_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        ret = mwifi_root_read(src_addr, &data_type, &data, &size, portMAX_DELAY);
        MDF_ERROR_GOTO(ret != MDF_OK, MEM_FREE, "<%s> mwifi_root_read", mdf_err_to_name(ret));
        //MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_root_recv", mdf_err_to_name(ret));
        if (data_type.upgrade){
            ret = mupgrade_root_handle(src_addr,data,size);
            MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mupgrade_root_handle", mdf_err_to_name(ret));
        } else {
            printf("BEFORE ROOT WRITE TASK: %lu\n", (unsigned long)esp_get_free_heap_size()); 
            ret = mesh_mqtt_write(src_addr, data, size, MESH_MQTT_DATA_JSON);
            MDF_ERROR_GOTO(ret != MDF_OK, MEM_FREE, "<%s> mesh_mqtt_publish", mdf_err_to_name(ret));
            MDF_LOGI("Receive [NODE] addr: " MACSTR ", size: %d, data: %s",MAC2STR(src_addr), size, data);
        }
        MEM_FREE:
            MDF_FREE(data);     
            printf("AFTER ROOT WRITE TASK: %lu\n", (unsigned long)esp_get_free_heap_size());   
    }
        
    MDF_LOGW("Root write task is exit");
    mesh_mqtt_stop();
    MDF_FREE(data);
    vTaskDelete(NULL);
}

static void root_read_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    size_t size_m = 0;
    char * data_m = NULL;

    uint8_t *data_1       = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    char name[32]       = {0x0};
    size_t total_size   = 0;
    int start_time      = 0;
    mupgrade_result_t upgrade_result = {0};
    mwifi_data_type_t data_type_1 = {.communicate = MWIFI_COMMUNICATE_MULTICAST};

    MDF_LOGI("Root read task is running");

    while (mwifi_is_connected() && esp_mesh_get_layer() == MESH_ROOT) {
        if (!mesh_mqtt_is_connect()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        mesh_mqtt_data_t *request = NULL;
        mwifi_data_type_t data_type = { 0x0 };
        
        /**
         * @brief Recv data from mqtt data queue, and forward to special device.
         */

        ret = mesh_mqtt_read(&request, pdMS_TO_TICKS(500));
        //MDF_LOGI("MESH_MQTT_READ");

        if (ret != MDF_OK) {
            //MDF_LOGE("MESH MQTT ERROR TIMEOUT");
            //MDF_ERROR_GOTO(ret != MDF_OK, MEM_FREE, "<%s> mwifi_root_write", mdf_err_to_name(ret));
            continue;
        }

        uint8_t mac[MWIFI_ADDR_LEN];
        
        MDF_LOGI("Root Received data from mqtt data queue");
        if (request->code == -2){
            MDF_LOGI("Election of a new Root");

            //mesh_vote_t* vote = (mesh_vote_t*)malloc(sizeof(mesh_vote_t));
            //vote->percentage = 0.8;
            //vote->is_rc_specified = false;

            ret = esp_mesh_waive_root(NULL, MESH_VOTE_REASON_ROOT_INITIATED);
            //ret = esp_mesh_waive_root(vote, MESH_VOTE_REASON_ROOT_INITIATED);
            if (ret != MDF_OK){
                printf("Error: %s\n",mdf_err_to_name(ret));
            }
            //MDF_ERROR_GOTO(ret != MDF_OK, MEM_FREE, "<%s> mwifi_root_write", mdf_err_to_name(ret));
        }


        else if (request->code == -3){
            int n = request->addrs_num;
            uint8_t dest_addr_update[n][MWIFI_ADDR_LEN];
            
            MDF_LOGI("Update"); 
            for (int i =0;i<n;i++){
                memcpy(mac,request->addrs_list + i * MWIFI_ADDR_LEN, MWIFI_ADDR_LEN);
                printf("Address: "MACSTR"\n",MAC2STR(mac));
                memcpy(dest_addr_update[i],mac,MWIFI_ADDR_LEN);
            }

            esp_http_client_config_t config = {
                .url            = CONFIG_FIRMWARE_UPGRADE_URL,
                .transport_type = HTTP_TRANSPORT_UNKNOWN,
            };
            printf("Opening Non-Volatile Storage (NVS) handle... ");
            esp_err_t err;
            nvs_handle_t my_handle_1;
            err = nvs_open("storage", NVS_READWRITE, &my_handle_1);
            if (err != ESP_OK) {
                printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            } else {
                printf("Done\n");
            }
            size_t size_4;
            //printf("first nvs_get_blob \n");
            err = nvs_get_str(my_handle_1, "Firmware_URL", NULL, &size_4);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
                printf("Error: %s\n", esp_err_to_name(err));
            }
            char* value_4 = malloc(size_4);
            //printf("second nvs_get_blob \n");
            err = nvs_get_str(my_handle_1, "Firmware_URL", value_4, &size_4);
            switch (err) {
                case ESP_OK:
                    printf("Done\n");
                    printf("Firmware URL= %s\n", value_4);
                break;
                case ESP_ERR_NVS_NOT_FOUND:
                    printf("The value is not initialized yet!\n");
                break;
                default :
                    printf("Error (%s) reading!\n", esp_err_to_name(err));
            }
            nvs_close(my_handle_1);
            vTaskDelay(pdMS_TO_TICKS(5000));
            config.url = value_4;
            //strcpy((char *)config.url, value_4);
            /**
            * @brief 1. Connect to the server
            */
           
            MDF_LOGI("CONNECT TO THE SERVER");
            esp_http_client_handle_t client = esp_http_client_init(&config);
            MDF_ERROR_GOTO(!client, EXIT, "Initialise HTTP connection");

            start_time = xTaskGetTickCount();

            MDF_LOGI("Open HTTP connection: %s", value_4);

            /**
            * @brief First, the firmware is obtained from the http server and stored on the root node.
            */
           
            do {
                ret = esp_http_client_open(client, 0);

                if (ret != MDF_OK) {
                    if (!esp_mesh_is_root()) {
                        MDF_LOGI("NODE IS NOT ROOT --> Go to EXIT");
                        goto EXIT;
                    }

                    vTaskDelay(pdMS_TO_TICKS(1000));
                    MDF_LOGW("<%s> Connection service failed", mdf_err_to_name(ret));
                }
            } while (ret != MDF_OK);

            total_size = esp_http_client_fetch_headers(client);
            sscanf(value_4, "%*[^//]//%*[^/]/%[^.bin]", name);
            free(value_4);

            if (total_size <= 0) {
                MDF_LOGW("Please check the address of the server");
                ret = esp_http_client_read(client, (char *)data_1, MWIFI_PAYLOAD_LEN);
                MDF_ERROR_GOTO(ret < 0, EXIT, "<%s> Read data from http stream", mdf_err_to_name(ret));

                MDF_LOGW("Recv data: %.*s", ret, data_1);
                MDF_LOGI("TOTAL SIZE <= 0 --> Go to EXIT");
                goto EXIT;
            }

            /**
            * @brief 2. Initialize the upgrade status and erase the upgrade partition.
            */
           
            MDF_LOGI("INITIALIZE THE UPGRADE STATUS AND ERASE THE UPGRADE PARTITION");
            ret = mupgrade_firmware_init(name, total_size);
            MDF_ERROR_GOTO(ret != MDF_OK, EXIT, "<%s> Initialize the upgrade status", mdf_err_to_name(ret));

            /**
            * @brief 3. Read firmware from the server and write it to the flash of the root node
            */
           
            MDF_LOGI("READ FIRMWARE FROM THE SERVER AND WRITE IT TO THE FLASH OF THE ROOT NODE");
            for (ssize_t size = 0, recv_size = 0; recv_size < total_size; recv_size += size) {
                size = esp_http_client_read(client, (char *)data_1, MWIFI_PAYLOAD_LEN);
                MDF_ERROR_GOTO(size < 0, EXIT, "<%s> Read data from http stream", mdf_err_to_name(ret));

                if (size > 0) {
                    
                /* @brief  Write firmware to flash */
                    
                    ret = mupgrade_firmware_download(data_1, size);
                    MDF_ERROR_GOTO(ret != MDF_OK, EXIT, "<%s> Write firmware to flash, size: %d, data: %.*s",
                                   mdf_err_to_name(ret), size, size, data_1);
                } else {
                    MDF_LOGI("SIZE > 0 --> Go to EXIT");
                    MDF_LOGW("<%s> esp_http_client_read", mdf_err_to_name(ret));
                    goto EXIT;
                }
            }

            MDF_LOGI("The service download firmware is complete, Spend time: %ds",
                     (xTaskGetTickCount() - start_time) * portTICK_RATE_MS / 1000);

            start_time = xTaskGetTickCount();

            /**
            * @brief 4. The firmware will be sent to each node.
            */
           
            MDF_LOGI("THE FIRMWARE WILL BE SENT TO EACH NODE");
            ret = mupgrade_firmware_send((uint8_t *)dest_addr_update, sizeof(dest_addr_update) / MWIFI_ADDR_LEN, &upgrade_result);
            MDF_ERROR_GOTO(ret != MDF_OK, EXIT, "<%s> mupgrade_firmware_send", mdf_err_to_name(ret));
            
            if (upgrade_result.successed_num == 0) {
                MDF_LOGI("dEVICE UPGRADE FAILED --> Go to EXIT");
                MDF_LOGW("Devices upgrade failed, unfinished_num: %d", upgrade_result.unfinished_num);
                goto EXIT;
            }

            MDF_LOGI("Firmware is sent to the device to complete, Spend time: %ds",
                     (xTaskGetTickCount() - start_time) * portTICK_RATE_MS / 1000);
            MDF_LOGI("Devices upgrade completed, successed_num: %d, unfinished_num: %d", upgrade_result.successed_num, upgrade_result.unfinished_num);

            /**
            * @brief 5. the root notifies nodes to restart
            */
           
            MDF_LOGI("THE ROOT NOTIFIES NODES TO RESTART");
            const char *restart_str = "restart";
            ret = mwifi_root_write(upgrade_result.successed_addr, upgrade_result.successed_num,
                                   &data_type_1, restart_str, strlen(restart_str), true);
            MDF_ERROR_GOTO(ret != MDF_OK, EXIT, "<%s> mwifi_root_recv", mdf_err_to_name(ret));

        EXIT:
            MDF_FREE(data_1);
            mupgrade_result_free(&upgrade_result);
            printf("erasing ota_1\n");
            nvs_flash_erase_partition("ota_1");
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            //vTaskDelete(NULL);

        } else {    
            printf("BEFORE ROOT READ TASK: %lu\n", (unsigned long)esp_get_free_heap_size());
            MDF_LOGI("Forward to device");
            size_m = asprintf(&data_m, "{\"msg_code\":%i,\"value\":%i}",request->code,request->value);
            // size_m = asprintf(&data_m, "{\"msg_code\":%i,\"value\":%i,\"I_O\":%i}",request->code,request->value,request->i_o);
            // printf("MSG CODE + VALUE + I_O\n");
        
            ret = mwifi_root_write(request->addrs_list, request->addrs_num, &data_type, data_m, size_m, true);
            MDF_FREE(data_m);
            MDF_ERROR_GOTO(ret != MDF_OK, MEM_FREE, "<%s> mwifi_root_write", mdf_err_to_name(ret));            
        }

MEM_FREE:

        MDF_FREE(request->addrs_list);
        MDF_FREE(request->data);
        MDF_FREE(request);
        printf("AFTER ROOT READ TASK: %lu\n", (unsigned long)esp_get_free_heap_size());
    }

    MDF_LOGW("Root read task is exit");
    //mesh_mqtt_stop();
    vTaskDelete(NULL);
}

static void node_read_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    char *data = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size = MWIFI_PAYLOAD_LEN;
    mwifi_data_type_t data_type = {0x0};
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0};
    uint8_t sta_mac[MWIFI_ADDR_LEN] = { 0 };
    char *sending_data = NULL;
    size_t size_data = 0;
    float *p;
    double Temp, Hum;

    MDF_LOGI("Node read task is running");
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);

    for(;;){
        if (!mwifi_is_connected()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        size = MWIFI_PAYLOAD_LEN;
        memset(data, 0, MWIFI_PAYLOAD_LEN);
        ret = mwifi_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_root_recv", mdf_err_to_name(ret));

        if (data_type.upgrade) { // This mesh package contains upgrade data.
            MDF_LOGI("MUPGRADE");
            ret = mupgrade_handle(src_addr, data, size);
            MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mupgrade_handle", mdf_err_to_name(ret));
        } else {
            MDF_LOGI("Receive [ROOT] addr: " MACSTR ", size: %d, data: %s",
                     MAC2STR(src_addr), size, data);

            /**
             * @brief Finally, the node receives a restart notification. Restart it yourself..
             */
            if (!strcmp(data, "restart")) {
                MDF_LOGI("Restart the version of the switching device");
                MDF_LOGW("The device will restart after 3 seconds");
                vTaskDelay(pdMS_TO_TICKS(3000));
                esp_restart();
            }
            else{
                MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_read", mdf_err_to_name(ret));
                MDF_LOGI("Node receive: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);

                cJSON *obj = cJSON_Parse(data);
                cJSON *msg_code = cJSON_GetObjectItem(obj, "msg_code");
                cJSON *value = cJSON_GetObjectItem(obj, "value");

                switch(msg_code->valueint){
                    case 0: 
                        AHT10_Init();
                        vTaskDelay(100 / portTICK_RATE_MS);
                        p = AHT10_Read_data();
                        Temp = (double) *(p + 1);
                        Temp = roundf(Temp * 10) / 10;
                        size_data = asprintf(&sending_data,"{\"Mac Address\": \"%02x%02x%02x%02x%02x%02x\",\"CH\":%d, \"VALUE\": \"%.1f\"}",
                                MAC2STR(sta_mac),0,Temp);
                        MDF_LOGD("Node send, size: %d, data: %s", size_data, sending_data);
                        ret = mwifi_write(NULL, &data_type, sending_data, size_data, true);
                        free(p);
                    break;

                    case 1:
                        AHT10_Init();
                        vTaskDelay(100 / portTICK_RATE_MS);
                        p = AHT10_Read_data();
                        Hum = (double) *(p + 0);
                        Hum = roundf(Hum * 10) / 10;
                        size_data = asprintf(&sending_data,"{\"Mac Address\": \"%02x%02x%02x%02x%02x%02x\",\"CH\":%d, \"VALUE\": \"%.1f\"}",
                                MAC2STR(sta_mac),1,Hum);
                        MDF_LOGD("Node send, size: %d, data: %s", size_data, sending_data);
                        ret = mwifi_write(NULL, &data_type, sending_data, size_data, true);
                        free(p);
                    break;

                    case 2:
                    printf("MSG CODE: %i\n", msg_code->valueint);
                    if (old_boiler == true){
                        // if ((((uint8_t) value->valueint) & 1) == 1){
                        if(value->valueint == 100){
                            printf("OLD BOILER ON\n");
                            gpio_set_level(RELE_PIN_ON, 1);
                            gpio_set_level(RELE_PIN_OFF, 0);
                            microsecondsDelay(10000);
                            gpio_set_level(RELE_PIN_ON, 0);
                            gpio_set_level(RELE_PIN_OFF, 0);
                            gpio_get_level(RELE_STATE);
                            printf("RELE STATE: %i",RELE_STATE);
                        } else{
                            printf("OLD BOILER OFF\n");
                            gpio_set_level(RELE_PIN_OFF, 1);
                            gpio_set_level(RELE_PIN_ON, 0);
                            microsecondsDelay(10000);
                            gpio_set_level(RELE_PIN_ON, 0);
                            gpio_set_level(RELE_PIN_OFF, 0);
                            gpio_get_level(RELE_STATE);
                            printf("RELE STATE: %i",RELE_STATE);
                        }
                    } else{
                        printf("Master and Slave Status flags\n");
                        Event_ID_0 = true;
                        StatusFlag = (uint8_t) value->valueint;
                    }
                    break;

                    case 3:
                        printf("MSG CODE: %i\n", msg_code->valueint);
                        if (old_boiler == false){
                            printf("Control setpoint ie CH water temperature setpoint (°C)\n");
                            Event_ID_1 = true;
                            Tset = (float) value->valuedouble;
                        }
                    break;

                    case 4:
                        printf("MSG CODE: %i\n", msg_code->valueint);
                        if (old_boiler == false){
                            printf("Application-specific fault flags and OEM fault code\n");
                            Event_ID_5 = true;
                        }
                    break;

                    case 5:
                        printf("MSG CODE: %i\n", msg_code->valueint);
                        if (old_boiler == false){
                            printf("Boiler flow water temperature (°C)\n");
                            Event_ID_25 = true;
                        }
                    break;

                    case 6:
                        printf("MSG CODE: %i\n", msg_code->valueint);
                        if (old_boiler == false){
                            printf("DHW temperature (°C)\n");
                            Event_ID_26 = true;
                        }
                    break;

                    case 7:
                        printf("MSG CODE: %i\n", msg_code->valueint);
                        if (old_boiler == false){
                            printf("Get DHW setpoint (°C) \n");
                            Event_ID_56_R = true;
                        }
                    break;
                    
                    case 8:
                        printf("MSG CODE: %i\n", msg_code->valueint);
                        if (old_boiler == false){
                            printf("Set DHW setpoint (°C) \n");
                            Event_ID_56_W = true;
                            TdhwSet = (float) value->valuedouble;
                        }
                    break;
                    
                    case 9:
                        printf("MSG CODE: %i\n", msg_code->valueint);
                        if (old_boiler == false){
                            printf("Get Max CH water setpoint (°C) \n");
                            Event_ID_57_R = true;
                        }
                    break;

                    case 10:
                        printf("MSG CODE: %i\n", msg_code->valueint);
                        if (old_boiler == false){
                            printf("Set Max CH water setpoint (°C) \n");
                            Event_ID_57_W = true;
                            MaxTset = (float) value->valuedouble;
                        }
                    break;

                    default:
                    printf("INVALID CODE");
                    break;
                }
                MDF_FREE(sending_data);
                cJSON_Delete(obj);
            }
        }
    }

    MDF_LOGW("Node read task is exit");
    MDF_FREE(data);
    vTaskDelete(NULL);
}

static void OpenTherm_Controller(void *arg){

    activateBoiler();
    unsigned long response;
    uint16_t response_value;
    float temp;
    mdf_err_t ret = MDF_OK;
    size_t size = 0;
    char *data = NULL;
    mwifi_data_type_t data_type = { 0x0 };
    uint8_t sta_mac[MWIFI_ADDR_LEN] = { 0 };
    int tmp_check = 0;
    char valueCon[20];

    while(1){
        tmp_check = 0;
        temp = 0;
        if (Event_ID_0 == true){
            response = sendRequest(insertParity(buildRequest(ReadData,0,StatusFlag,0)));
            response_value = ((response & (255)) | (response & (255<<8)));
            esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
            size = asprintf(&data, "{\"Mac Address\": \"%02x%02x%02x%02x%02x%02x\",\"CH\":%d, \"VALUE\": \"{Master_Flag:%u, Slave_Flag:%u}\"}",
                            MAC2STR(sta_mac),2,(response_value>>8) & 255, response_value & 255);
            MDF_LOGD("Node send, size: %d, data: %s", size, data);
            ret = mwifi_write(NULL, &data_type, data, size, true);
            MDF_FREE(data);
            Event_ID_0 = false;
        } else{
            // tmp_time = esp_timer_get_time();
            response = sendRequest(insertParity(buildRequest(ReadData,0,StatusFlag,0)));
            response_value = ((response & (255)) | (response & (255<<8)));
        }
        tmp_check = tmp_check + response;
        vTaskDelay(300 / portTICK_PERIOD_MS); //950


        if (Event_ID_1 == true){
            response = sendRequest(insertParity(buildRequest(WriteData,1,((toF88(Tset)>>8) & 255),(toF88(Tset) & 255))));
            response_value = ((response & (255)) | (response & (255<<8))); 
            temp = fromF88(response_value);           
            esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
            //sprintf(valueCon, "%f",temp);
            size = asprintf(&data, "{\"Mac Address\": \"%02x%02x%02x%02x%02x%02x\",\"CH\":%d,\"VALUE\":\"%f\"}",
                            MAC2STR(sta_mac),3,temp);
            MDF_LOGD("Node send, size: %d, data: %s", size, data);
            ret = mwifi_write(NULL, &data_type, data, size, true);
            MDF_FREE(data);
            Event_ID_1 = false;
        } else{
            // printf("DELTA: %d\n",(int) (esp_timer_get_time() - tmp_time));
            // tmp_time = esp_timer_get_time();
            response = sendRequest(insertParity(buildRequest(WriteData,1,((toF88(Tset)>>8) & 255),(toF88(Tset) & 255))));
            response_value = ((response & (255)) | (response & (255<<8)));
        }
        tmp_check = tmp_check + response;
        vTaskDelay(300 / portTICK_PERIOD_MS); //950

        if (Event_ID_5 == true){
            response = sendRequest(insertParity(buildRequest(ReadData,5,0,0)));
            response_value = ((response & (255)) | (response & (255<<8)));
            temp = fromF88(response_value); 
            esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
            size = asprintf(&data, "{\"Mac Address\": \"%02x%02x%02x%02x%02x%02x\",\"CH\":%d, \"VALUE\": \"{Fault_Flag:%u, OEM_code:%u}\"}",
                            MAC2STR(sta_mac),4,(response_value>>8) & 255, response_value & 255);
            MDF_LOGD("Node send, size: %d, data: %s", size, data);
            ret = mwifi_write(NULL, &data_type, data, size, true);
            MDF_FREE(data);
            Event_ID_5 = false;
        } else{
            // printf("DELTA: %d\n", (int)(esp_timer_get_time() - tmp_time));
            // tmp_time = esp_timer_get_time();
            response = sendRequest(insertParity(buildRequest(ReadData,5,0,0)));
            response_value = ((response & (255)) | (response & (255<<8)));
        }
        tmp_check = tmp_check + response;
        vTaskDelay(300 / portTICK_PERIOD_MS);//950


        if (Event_ID_25 == true){
            response = sendRequest(insertParity(buildRequest(ReadData,25,0,0)));
            response_value = ((response & (255)) | (response & (255<<8)));
            temp = fromF88(response_value); 
            esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
            //sprintf(valueCon, "%f",temp);
            size = asprintf(&data, "{\"Mac Address\": \"%02x%02x%02x%02x%02x%02x\",\"CH\":%d,\"VALUE\":\"%f\"}",
                            MAC2STR(sta_mac),5,temp);
            MDF_LOGD("Node send, size: %d, data: %s", size, data);
            ret = mwifi_write(NULL, &data_type, data, size, true);
            MDF_FREE(data);
            Event_ID_25 = false;
        } else{
            // printf("DELTA: %d\n", (int)(esp_timer_get_time() - tmp_time));
            // tmp_time = esp_timer_get_time();
            response = sendRequest(insertParity(buildRequest(ReadData,25,0,0)));
            response_value = ((response & (255)) | (response & (255<<8)));
        }
        tmp_check = tmp_check + response;
        vTaskDelay(300 / portTICK_PERIOD_MS); //950

        if (Event_ID_26 == true){
            response = sendRequest(insertParity(buildRequest(ReadData,26,0,0)));
            response_value = ((response & (255)) | (response & (255<<8)));
            temp = fromF88(response_value); 
            esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
            //sprintf(valueCon, "%f",temp);
            size = asprintf(&data, "{\"Mac Address\": \"%02x%02x%02x%02x%02x%02x\",\"CH\":%d,\"VALUE\":\"%f\"}",
                            MAC2STR(sta_mac),6,temp);
            MDF_LOGD("Node send, size: %d, data: %s", size, data);
            ret = mwifi_write(NULL, &data_type, data, size, true);
            MDF_FREE(data);
            Event_ID_26 = false;
        }else{
            // printf("DELTA: %d\n", (int)(esp_timer_get_time() - tmp_time));
            // tmp_time = esp_timer_get_time();
            response = sendRequest(insertParity(buildRequest(ReadData,26,0,0)));
            response_value = ((response & (255)) | (response & (255<<8)));
        }
        tmp_check = tmp_check + response;
        vTaskDelay(300 / portTICK_PERIOD_MS); //950


        if (Event_ID_56_R == true){
            response = sendRequest(insertParity(buildRequest(ReadData,56,0,0)));
            response_value = ((response & (255)) | (response & (255<<8)));      
            temp = fromF88(response_value);       
            esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
            //sprintf(valueCon, "%f",temp);
            size = asprintf(&data, "{\"Mac Address\": \"%02x%02x%02x%02x%02x%02x\",\"CH\":%d,\"VALUE\":\"%f\"}",
                            MAC2STR(sta_mac),7,temp);
            MDF_LOGD("Node send, size: %d, data: %s", size, data);
            ret = mwifi_write(NULL, &data_type, data, size, true);
            MDF_FREE(data);
            Event_ID_56_R = false;
        } else{
            // printf("DELTA: %d\n", (int)(esp_timer_get_time() - tmp_time));
            // tmp_time = esp_timer_get_time();
            response = sendRequest(insertParity(buildRequest(ReadData,56,0,0)));
            response_value = ((response & (255)) | (response & (255<<8)));
        }
        tmp_check = tmp_check + response;
        vTaskDelay(300 / portTICK_PERIOD_MS); //950

        if (Event_ID_57_R == true){
            response = sendRequest(insertParity(buildRequest(ReadData,57,0,0)));
            response_value = ((response & (255)) | (response & (255<<8)));
            temp = fromF88(response_value); 
            esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
            //sprintf(valueCon, "%f",temp);
            size = asprintf(&data, "{\"Mac Address\": \"%02x%02x%02x%02x%02x%02x\",\"CH\":%d,\"VALUE\":\"%f\"}",
                            MAC2STR(sta_mac),9,temp);
            MDF_LOGD("Node send, size: %d, data: %s", size, data);
            ret = mwifi_write(NULL, &data_type, data, size, true);
            MDF_FREE(data);
            Event_ID_57_R = false;
        }else{
            // printf("DELTA: %d\n", (int)(esp_timer_get_time() - tmp_time));
            // tmp_time = esp_timer_get_time();
            response = sendRequest(insertParity(buildRequest(ReadData,57,0,0)));
            response_value = ((response & (255)) | (response & (255<<8)));
        }
        tmp_check = tmp_check + response;
        vTaskDelay(300 / portTICK_PERIOD_MS);//950

        if (Event_ID_56_W == true){
            response = sendRequest(insertParity(buildRequest(WriteData,56,((toF88(TdhwSet)>>8) & 255),(toF88(TdhwSet) & 255))));
            response_value = ((response & (255)) | (response & (255<<8)));
            temp = fromF88(response_value); 
            esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
            //sprintf(valueCon, "%f",temp);
            size = asprintf(&data, "{\"Mac Address\": \"%02x%02x%02x%02x%02x%02x\",\"CH\":%d,\"VALUE\":\"%f\"}",
                            MAC2STR(sta_mac),8,temp);
            MDF_LOGD("Node send, size: %d, data: %s", size, data);
            ret = mwifi_write(NULL, &data_type, data, size, true);
            MDF_FREE(data);
            Event_ID_56_W = false;
        }else{
            // printf("DELTA: %d\n",(int) (esp_timer_get_time() - tmp_time));
            // tmp_time = esp_timer_get_time();
            response = sendRequest(insertParity(buildRequest(WriteData,56,((toF88(TdhwSet)>>8) & 255),(toF88(TdhwSet) & 255))));
            response_value = ((response & (255)) | (response & (255<<8)));
        }
        tmp_check = tmp_check + response;
        vTaskDelay(300 / portTICK_PERIOD_MS); //950

        if (Event_ID_57_W == true){
            response = sendRequest(insertParity(buildRequest(WriteData,56,((toF88(MaxTset)>>8) & 255),(toF88(MaxTset) & 255))));
            response_value = ((response & (255)) | (response & (255<<8)));
            temp = fromF88(response_value); 
            esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
            //sprintf(valueCon, "%f",temp);
            size = asprintf(&data, "{\"Mac Address\": \"%02x%02x%02x%02x%02x%02x\",\"CH\":%d,\"VALUE\":\"%f\"}",
                            MAC2STR(sta_mac),10,temp);
            MDF_LOGD("Node send, size: %d, data: %s", size, data);
            ret = mwifi_write(NULL, &data_type, data, size, true);
            MDF_FREE(data);
            Event_ID_57_W = false;
        }else{
            // printf("DELTA: %d\n",(int) (esp_timer_get_time() - tmp_time));
            response = sendRequest(insertParity(buildRequest(WriteData,56,((toF88(MaxTset)>>8) & 255),(toF88(MaxTset) & 255))));
            response_value = ((response & (255)) | (response & (255<<8)));
        }
        tmp_check = tmp_check + response;
        vTaskDelay(300 / portTICK_PERIOD_MS);  //950
        if (tmp_check == 0){
            if(old_boiler == false){
                printf("Controller in Rele Mode\n");
            }
            openTherm = false;
            old_boiler = true;
        }else{
            if(openTherm == false){
                printf("Controller in OpenTherm Mode\n");
            }
            old_boiler = false;
            openTherm = true;
        }
    }
    MDF_LOGW("Node task is exit");
    vTaskDelete(NULL);
}

static void Sensing_Temp_Hum(void *arg)
{
    mdf_err_t ret = MDF_OK;
    size_t size = 0;
    char *data = NULL;
    mwifi_data_type_t data_type = { 0x0 };
    uint8_t sta_mac[MWIFI_ADDR_LEN] = { 0 };
    mesh_addr_t parent_mac = { 0 };
    float *p;
    double Temp, Hum;
    double tmpTemp = 0;

    MDF_LOGI("Sensing Temp and Hum task is running");

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    
    for (;;) {
        if (!mwifi_is_connected() || !mwifi_get_root_status()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        /**
         * @brief Send device information to mqtt server throught root node.
         */
        
	    AHT10_Init();
	    vTaskDelay(100 / portTICK_RATE_MS);
	    p = AHT10_Read_data();
        Hum = (double) *(p + 0);
        Temp = (double) *(p + 1);
        Hum = roundf(Hum * 10) / 10;
        Temp = roundf(Temp * 10) / 10;

        if (fabs(Temp - tmpTemp) >= 0.1){
            esp_mesh_get_parent_bssid(&parent_mac);

            size = asprintf(&data,"{\"Mac Address\": \"%02x%02x%02x%02x%02x%02x\",\"CH\":%d, \"VALUE\": \"%.1f\"}",
                            MAC2STR(sta_mac),0,Temp);
            MDF_LOGD("Node send, size: %d, data: %s", size, data);
            ret = mwifi_write(NULL, &data_type, data, size, true);

            size = asprintf(&data,"{\"Mac Address\": \"%02x%02x%02x%02x%02x%02x\",\"CH\":%d, \"VALUE\": \"%.1f\"}",
                            MAC2STR(sta_mac),1,Hum);
            MDF_LOGD("Node send, size: %d, data: %s", size, data);
            ret = mwifi_write(NULL, &data_type, data, size, true);

            MDF_FREE(data);
            free(p);
            MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
        }
        tmpTemp = Temp;        
        vTaskDelay(10000 / portTICK_RATE_MS);
    }

    MDF_LOGW("Node task is exit");
    vTaskDelete(NULL);
}

static mdf_err_t wifi_init()
{
    mdf_err_t ret = nvs_flash_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        MDF_ERROR_ASSERT(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    MDF_ERROR_ASSERT(ret);

    MDF_ERROR_ASSERT(esp_netif_init());
    //MDF_ERROR_ASSERT(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&sta_netif, NULL));
    MDF_ERROR_ASSERT(esp_wifi_init(&cfg));
    MDF_ERROR_ASSERT(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    MDF_ERROR_ASSERT(esp_wifi_set_mode(WIFI_MODE_STA));
    MDF_ERROR_ASSERT(esp_wifi_set_ps(WIFI_PS_NONE));
    MDF_ERROR_ASSERT(esp_mesh_set_6m_rate(false));
    MDF_ERROR_ASSERT(esp_wifi_start());

    return MDF_OK;
}

/**
 * @brief All module events will be sent to this task in esp-mdf
 *
 * @Note:
 *     1. Do not block or lengthy operations in the callback function.
 *     2. Do not consume a lot of memory in the callback function.
 *        The task memory of the callback function is only 4KB.
 */
static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx)
{
    MDF_LOGI("event_loop_cb, event: %d", event);

    switch (event) {
        case MDF_EVENT_MWIFI_STARTED:
            MDF_LOGI("MESH is started");
            break;

        case MDF_EVENT_MWIFI_PARENT_CONNECTED:
            MDF_LOGI("Parent is connected on station interface");

            if (esp_mesh_is_root()) {
                esp_netif_dhcpc_start(sta_netif);
            }
            
            break;

        case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
            MDF_LOGI("Parent is disconnected on station interface");
            vTaskDelay(3000 / portTICK_RATE_MS);
            if (esp_mesh_is_root()) {
                mesh_mqtt_stop();
            }

            break;

        case MDF_EVENT_MWIFI_ROUTING_TABLE_ADD:
        
        case MDF_EVENT_MWIFI_ROUTING_TABLE_REMOVE:
            MDF_LOGI("MDF_EVENT_MWIFI_ROUTING_TABLE_REMOVE, total_num: %d", esp_mesh_get_total_node_num());

            if (esp_mesh_is_root() && mwifi_get_root_status()) {
                mdf_err_t err = mesh_mqtt_update_topo();

                if (err != MDF_OK) {
                    MDF_LOGE("Update topo failed");
                }
            }

            break;

        case MDF_EVENT_MWIFI_ROOT_GOT_IP: {
            //gpio_set_level(GPIO_ROOT, 1);
            MDF_LOGI("Root obtains the IP address. It is posted by LwIP stack automatically");
                        printf("Opening Non-Volatile Storage (NVS) handle... ");
            nvs_handle_t my_handle_2;
            esp_err_t err;
            err = nvs_open("storage", NVS_READWRITE, &my_handle_2);
            if (err != ESP_OK) {
                printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            } else {
                printf("Done\n");
            }
            size_t size_5;
            //printf("first nvs_get_blob \n");
            err = nvs_get_str(my_handle_2, "MQTT_URL", NULL, &size_5);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
            char* value_5 = malloc(size_5);
            //printf("second nvs_get_blob \n");
            err = nvs_get_str(my_handle_2, "MQTT_URL", value_5, &size_5);
            switch (err) {
                case ESP_OK:
                    printf("Done\n");
                    printf("MQTT URL = %s\n", value_5);
                break;
                case ESP_ERR_NVS_NOT_FOUND:
                    printf("The value is not initialized yet!\n");
                break;
                default :
                    printf("Error (%s) reading!\n", esp_err_to_name(err));
            }
            nvs_close(my_handle_2);
            
            mesh_mqtt_start(value_5); 
            free(value_5);

            xTaskCreate(root_write_task, "root_write", 4 * 1024,
                    NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);  
            xTaskCreate(root_read_task, "root_read", 4 * 1024,
                    NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);            
            break;
        }

        case MDF_EVENT_CUSTOM_MQTT_CONNECT:
            MDF_LOGI("MQTT connect");
            mdf_err_t err = mesh_mqtt_update_topo();
            if (err != MDF_OK) {
                MDF_LOGE("Update topo failed");
            }
            err = mesh_mqtt_subscribe();
            if (err != MDF_OK) {
                MDF_LOGE("Subscribe failed");
            }
            mwifi_post_root_status(true);
            break;

        case MDF_EVENT_CUSTOM_MQTT_DISCONNECT:
            //gpio_set_level(GPIO_ROOT, 0);
            MDF_LOGI("MQTT disconnected");
            mwifi_post_root_status(false);
            break;

        case MDF_EVENT_MUPGRADE_STARTED: {
            mupgrade_status_t status = {0x0};
            mupgrade_get_status(&status);
            MDF_LOGI("MDF_EVENT_MUPGRADE_STARTED, name: %s, size: %d",
                     status.name, status.total_size);
            break;
        }

        case MDF_EVENT_MUPGRADE_STATUS:
            MDF_LOGI("Upgrade progress: %d%%", (int)ctx);
            break;

        default:
            break;
    }

    return MDF_OK;
}

void main_task_mesh(){

        vTaskDelay(5000 / portTICK_RATE_MS);
        printf("\n");
        printf("Opening Non-Volatile Storage (NVS) handle... ");
        nvs_handle_t my_handle;
        esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
        if (err != ESP_OK) {
            printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        } else {
            printf("Done\n");
        }
        size_t size;
            //printf("first nvs_get_blob \n");
            err = nvs_get_str(my_handle, "mesh_id", NULL, &size);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
                printf("Error: %s\n", esp_err_to_name(err));
            }
            char* value = malloc(size);
            //printf("second nvs_get_blob \n");
            err = nvs_get_str(my_handle, "mesh_id", value, &size);
            switch (err) {
                case ESP_OK:
                    printf("Done\n");
                    printf("MESH ID = %s\n", value);
                break;
                case ESP_ERR_NVS_NOT_FOUND:
                    printf("The value is not initialized yet!\n");
                break;
                default :
                    printf("Error (%s) reading!\n", esp_err_to_name(err));
            }

            size_t size_1;
            //printf("first nvs_get_blob \n");
            err = nvs_get_str(my_handle, "mesh_password", NULL, &size_1);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
                printf("Error: %s\n", esp_err_to_name(err));
            }
            char* value_1 = malloc(size_1);
            //printf("second nvs_get_blob \n");
            err = nvs_get_str(my_handle, "mesh_password", value_1, &size_1);
            switch (err) {
                case ESP_OK:
                    printf("Done\n");
                    printf("MESH PASSWORD = %s\n", value_1);
                break;
                case ESP_ERR_NVS_NOT_FOUND:
                    printf("The value is not initialized yet!\n");
                break;
                default :
                    printf("Error (%s) reading!\n", esp_err_to_name(err));
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
            mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
            mwifi_config_t config = {
                .router_ssid = CONFIG_GATEWAY_ESSID,
                .router_password = CONFIG_GATEWAY_PASSWORD,
                .mesh_id = CONFIG_MESH_ID,
                .mesh_password = CONFIG_MESH_PASSWORD,
                //.mesh_type = CONFIG_DEVICE_TYPE,
            };
            strcpy((char *)config.mesh_id, value);
            strcpy((char *)config.mesh_password, value_1);
            strcpy((char *)config.router_ssid, value_2);
            strcpy((char *)config.router_password, value_3);

            free(value);
            free(value_1);
            free(value_2);
            free(value_3);
    /**
     * @brief Set the log level for serial port printing.
     */
            esp_log_level_set("*", ESP_LOG_INFO);
            esp_log_level_set(TAG, ESP_LOG_DEBUG);
            esp_log_level_set("mesh_mqtt", ESP_LOG_DEBUG);
            esp_log_level_set("mupgrade_node", ESP_LOG_DEBUG);
            esp_log_level_set("mupgrade_root", ESP_LOG_DEBUG);

    /**
     * @brief Initialize wifi mesh.
     */
            MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
            MDF_ERROR_ASSERT(wifi_init());
            MDF_ERROR_ASSERT(mwifi_init(&cfg));
            MDF_ERROR_ASSERT(mwifi_set_config(&config));
            MDF_ERROR_ASSERT(mwifi_start());

    /**
     * @brief Create node handler
     */
    xTaskCreate(Sensing_Temp_Hum, "Sensing_Temp_Hum", 4 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    xTaskCreate(node_read_task, "node_read_task", 4 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    xTaskCreate(OpenTherm_Controller, "OpenTherm_Controller", 4 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    
}

void app_main()
{	    

    gpio_pad_select_gpio(OT_OUT_PIN);
    gpio_pad_select_gpio(OT_IN_PIN);

    gpio_pad_select_gpio(RELE_PIN_ON);
    gpio_pad_select_gpio(RELE_PIN_OFF);
    gpio_pad_select_gpio(RELE_STATE);
    gpio_set_pull_mode(RELE_STATE, GPIO_PULLUP_ONLY);

    gpio_set_direction(OT_OUT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(OT_IN_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(OT_IN_PIN, GPIO_PULLDOWN_ONLY);

    gpio_set_direction(RELE_STATE, GPIO_MODE_INPUT);
    gpio_set_direction(RELE_PIN_ON, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELE_PIN_OFF, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_PIN_ON,0);
    gpio_set_level(RELE_PIN_OFF,0);

    gpio_pad_select_gpio(13);
    gpio_set_direction(13, GPIO_MODE_OUTPUT);
    gpio_set_level(13, 1);
    AHT10_initialization();

    /*
    gpio_pad_select_gpio(GPIO_ROOT);
    gpio_set_direction(GPIO_ROOT, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_ROOT, 0);
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
    nvs_close(my_handle);
    configured = boot_start;
    wifi_init_sta();  
}

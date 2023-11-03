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
#include "mbedtls/md5.h"
#include <aht.h>
#include "cJSON.h"
#include "esp_event_loop.h"
#include <stdint.h>
#include <stddef.h>
#include "esp_netif.h"
//#include "protocol_examples_common.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
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

#define ADDR AHT_I2C_ADDRESS_GND
#define AHT_TYPE AHT_TYPE_AHT20
#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define MD5_MAX_LEN 16

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

uint32_t MQTT_CONNEECTED = 0;

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
esp_netif_t *sta;
esp_event_handler_instance_t instance_any_id;
esp_event_handler_instance_t instance_got_ip;
int32_t configured = 0;
static const char *TAG = "STE_Direct";
int32_t sending_period_global = 30;
float delta_temperature_global = 0.5;
float offset_temperature_global = 0.0;
int32_t boiler_status_global = 0;
float Temp_Global, Hum_Global;
esp_mqtt_client_handle_t client = NULL;

static void main_routine();

static const char REQUEST[512] = "POST " WEB_URL " HTTP/1.0\r\n"
    "User-Agent: esp-idf/1.0 esp32\r\n"
    "Host: "WEB_SERVER"\r\n"
    "Content-Type: application/json\r\n"
    "Content-Length: 35\r\n"
    "\r\n"
    "{\"macAddress\": %s}";

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
            .url = "http://192.168.5.1:9001/api/firmware/STE",
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
        case 0: 
            size_data = asprintf(&sending_data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\",\"channel\":%d, \"Temp\": \"%.1f\", \"Hum\": \"%.1f\", \"Period\": %d, \"Delta_T\": \"%.1f\", \"Offset_T\": \"%.1f\"}",
                MAC2STR(sta_mac),0,Temp_Global + offset_temperature_global,Hum_Global,sending_period_global,delta_temperature_global,offset_temperature_global);
            if(MQTT_CONNEECTED){
                esp_mqtt_client_publish(client, "mesh/toCloud", sending_data, 0, 0, 0);
                ESP_LOGI(TAG,"Node send, size: %d, data: %s", size_data, sending_data);
            }
            free(sending_data);   
        break;
        case 1:
            if(sending_period_global != value_h){
                esp_err_t err;
                nvs_handle_t my_handle;
                err = nvs_open("storage", NVS_READWRITE, &my_handle);
                if (err != ESP_OK) {
                    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
                } else {
                    printf("Done\n");
                }
                sending_period_global = value_h;
                printf("Setting Sending Period\n");
                err = nvs_set_i32(my_handle,"sending_period",sending_period_global);
                printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                printf("Committing updates in NVS ... ");
                err = nvs_commit(my_handle);
                nvs_close(my_handle);
            }
        break;

        case 2:
            if(delta_temperature_global != value_h + ((float) value_l /10)){
                esp_err_t err_1;
                nvs_handle_t my_handle_1;
                err_1 = nvs_open("storage", NVS_READWRITE, &my_handle_1);
                if (err_1 != ESP_OK) {
                    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err_1));
                } else {
                    printf("Done\n");
                }
                delta_temperature_global = value_h + ((float) value_l /10); 
                printf("Setting Sending Period\n");
                err_1 = nvs_set_i32(my_handle_1,"delta_H",value_h);
                printf((err_1 != ESP_OK) ? "Failed!\n" : "Done\n");
                err_1 = nvs_set_i32(my_handle_1,"delta_L",value_l);
                printf((err_1 != ESP_OK) ? "Failed!\n" : "Done\n");
                printf("Committing updates in NVS ... ");
                err_1 = nvs_commit(my_handle_1);
                nvs_close(my_handle_1);
            }
        break;
        case 3:
            if(offset_temperature_global != value_h + ((float) value_l /10)){
                esp_err_t err_1;
                nvs_handle_t my_handle_1;
                err_1 = nvs_open("storage", NVS_READWRITE, &my_handle_1);
                if (err_1 != ESP_OK) {
                    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err_1));
                } else {
                    printf("Done\n");
                }
                offset_temperature_global = value_h + ((float) value_l /10); 
                printf("Setting Sending Period\n");
                err_1 = nvs_set_i32(my_handle_1,"offset_H",value_h);
                printf((err_1 != ESP_OK) ? "Failed!\n" : "Done\n");
                err_1 = nvs_set_i32(my_handle_1,"offset_L",value_l);
                printf((err_1 != ESP_OK) ? "Failed!\n" : "Done\n");
                printf("Committing updates in NVS ... ");
                err_1 = nvs_commit(my_handle_1);
                nvs_close(my_handle_1);
            }
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
        printf("INVALID CODE\n"); 
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
                if(channel == 0 || channel == -2){
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

static void event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data)
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
        ESP_ERROR_CHECK(esp_wifi_start()); 
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


static void Sensing_Temp_Hum(void *arg)
{
    size_t size = 0;
    char *data = NULL;
    uint8_t sta_mac[6] = {0};
    float Temp, Hum;
    float tmpTemp = 0;
    uint8_t sendingTimes = 0;
    
    //MDF_LOGI("Sensing Temp and Hum task is running");

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    
    for (;;) {

        //Temp = Temp_Global + offset_temperature_global;
        Temp = (Temp_Global + offset_temperature_global - 2.3) * 1.94 - 22.43;
        Hum = Hum_Global;
        size = asprintf(&data,"{\"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\",\"channel\":%d, \"Temp\": \"%.1f\", \"Hum\": \"%.1f\", \"Period\": %d, \"Delta_T\": \"%.1f\", \"Offset_T\": \"%.1f\"}",
                            MAC2STR(sta_mac),0,Temp,Hum,sending_period_global,delta_temperature_global,offset_temperature_global);

        if ((fabs(Temp - tmpTemp) >= delta_temperature_global) || sendingTimes >= sending_period_global - 5){
 
            //MDF_LOGD("Node send, size: %d, data: %s", size, data);
            if(MQTT_CONNEECTED){
                esp_mqtt_client_publish(client, "mesh/toCloud", data, 0, 0, 0);
            }
            free(data);
            //MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
            tmpTemp = Temp; 
            sendingTimes = 0;

        }
        else{
            free(data);
            sendingTimes = sendingTimes + 5;
        }               
        vTaskDelay(5000 / portTICK_RATE_MS);
    }
    //MDF_LOGW("Node task is exit");
    vTaskDelete(NULL);
}

void task(void *pvParameters)
{
    aht_t dev = { 0 };
    dev.mode = AHT_MODE_NORMAL;
    dev.type = AHT_TYPE_AHT20;

    ESP_ERROR_CHECK(aht_init_desc(&dev, ADDR, 0, 23, 22));
    vTaskDelay(2000 / portTICK_RATE_MS);
    ESP_ERROR_CHECK(aht_init(&dev));

    bool calibrated;
    ESP_ERROR_CHECK(aht_get_status(&dev, NULL, &calibrated));
    if (calibrated)
        ESP_LOGI(TAG, "Sensor calibrated");
    else
        ESP_LOGW(TAG, "Sensor not calibrated!");

    float temperature, humidity;

    while (1)
    {
        esp_err_t res = aht_get_data(&dev, &temperature, &humidity);
        if (res == ESP_OK){
            //ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.2f%%", temperature, humidity);
            Temp_Global = temperature;
            Hum_Global = humidity;
        }
        else{
            ESP_LOGE(TAG, "Error reading data: %d (%s)", res, esp_err_to_name(res));
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void main_routine(){

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

    gpio_pad_select_gpio(13);
    gpio_set_direction(13, GPIO_MODE_OUTPUT);
    gpio_set_level(13, 1);

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreatePinnedToCore(task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    
    xTaskCreate(Sensing_Temp_Hum, "Sensing_Temp_Hum", 4 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
}

void app_main()
{	
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

    printf("Reading Sending Period from NVS ... ");
    int32_t boot_start = 0; 
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

    printf("Reading Sending Period from NVS ... ");
    int32_t sending_period = 30; 
    err = nvs_get_i32(my_handle, "sending_period", &sending_period);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Sending Period Variable = %d\n", sending_period);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to the Default Value\n");
            err = nvs_set_i32(my_handle,"sending_period",sending_period);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
    sending_period_global = sending_period;
        
    printf("Reading Delta Temperature HIGH from NVS ... ");
    int32_t delta_H = 0; 
    int32_t delta_L = 5; 
    err = nvs_get_i32(my_handle, "delta_H", &delta_H);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Delta Temp MSB Variable = %d\n", delta_H);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to Default Value\n");
            err = nvs_set_i32(my_handle,"delta_H",delta_H);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    printf("Reading Delta Temperature  LOW from NVS ... ");
    err = nvs_get_i32(my_handle, "delta_L", &delta_L);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Delta Temp LSB Variable = %d\n", delta_L);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to the Default Value\n");
            err = nvs_set_i32(my_handle,"delta_L",delta_L);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    delta_temperature_global = delta_H + ((float)delta_L / 10);

    printf("Reading Offset MSB from NVS ... ");
    int32_t offset_H = 0; 
    int32_t offset_L = 0; 
    err = nvs_get_i32(my_handle, "offset_H", &offset_H);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Offset MSB Variable = %d\n", offset_H);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to Default Value\n");
            err = nvs_set_i32(my_handle,"offset_H",offset_H);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    printf("Reading Delta Temperature  LOW from NVS ... ");
    err = nvs_get_i32(my_handle, "offset_L", &offset_L);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Delta Temp LSB Variable = %d\n", offset_L);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Setting the value to the Default Value\n");
            err = nvs_set_i32(my_handle,"offset_L",offset_L);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    offset_temperature_global = offset_H + ((float)offset_L / 10);
    printf("DELTA TEMPERATURE = %f\n",delta_temperature_global);
    printf("OFFSET TEMPERATURE = %f\n",offset_temperature_global);
    printf("Global Sending Period Variable = %d\n",sending_period_global);
    
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

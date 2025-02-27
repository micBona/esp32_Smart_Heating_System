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

#include "mesh_mqtt_handle.h"
#include "cJSON.h"
#include "esp_wifi.h"
#include "mbedtls/base64.h"
#include "mlink.h"
#include "mwifi.h"

static struct mesh_mqtt {
    xQueueHandle queue; /**< mqtt receive data queue */
    esp_mqtt_client_handle_t client; /**< mqtt client */
    bool is_connected;
    uint8_t addr[MWIFI_ADDR_LEN];
    char publish_topic[32];
    char topo_topic[32];
} g_mesh_mqtt;

static const char *TAG = "mesh_mqtt";

static const char publish_topic_template[] = "mesh/toCloud";

static const char subscribe_topic_template[] = "mesh/toDevice";
//static uint8_t mwifi_addr_any[] = MWIFI_ADDR_ANY;

static mesh_mqtt_data_t *mesh_mqtt_parse_data(const char *topic, size_t topic_size, const char *payload, size_t payload_size)
{
    MDF_LOGE("mesh mqtt parse data");
    uint8_t mac[MWIFI_ADDR_LEN];

    char *str = MDF_MALLOC(payload_size + 1);

    if (str == NULL) {
        MDF_LOGE("No memory");
        return NULL;
    }

    memcpy(str, payload, payload_size);
    str[payload_size] = '\0';
    cJSON *obj = cJSON_Parse(str);
    MDF_FREE(str);

    if (obj == NULL) {
        MDF_LOGE("Parse JSON error");
        return NULL;
    }

    mesh_mqtt_data_t *request = MDF_CALLOC(1, sizeof(mesh_mqtt_data_t));

    if (request == NULL) {
        MDF_LOGE("No memory");
        goto _exit;
    }
        
    cJSON *addr = cJSON_GetObjectItem(obj, "mac");
    //printf("MAC ADDRESS: %s\n",addr->valuestring);

    //mapping da aa:bb:cc:dd:ee:ff a aabbccddeeff
    char * macAddressString = addr->valuestring;
    char macString [13];
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
    //printf("MAC ADDRESSSSS: %s\n",macString);

    cJSON *code = cJSON_GetObjectItem(obj, "channel");
    if (code == NULL) {
        MDF_LOGE("DATA IS NULL --> GOTO EXIT");
        MDF_FREE(request->addrs_list);
        MDF_FREE(request);
        goto _exit;
    }
    request->code = code->valueint;

    cJSON *value = cJSON_GetObjectItem(obj, "value");

    if(code->valueint == -3){
        printf("OTA MESSAGE\n"); 
        if (value == NULL || cJSON_IsArray(value) != true) {
            MDF_LOGE("value == NULL || cJSON-IsArray(value) != true --> GOTO EXIT");
            MDF_FREE(request->addrs_list);
            MDF_FREE(request);
            goto _exit;
        }
        request->addrs_num = cJSON_GetArraySize(value);
        request->addrs_list = MDF_MALLOC(MWIFI_ADDR_LEN * request->addrs_num);
        cJSON *item = NULL;
        int i = 0;
        cJSON_ArrayForEach(item, value) {
            if (cJSON_IsString(item) != true) {
                 MDF_LOGE("cJSON_IsString(item) != true --> GOTO EXIT");
                 MDF_FREE(request->addrs_list);
                 MDF_FREE(request);
                 goto _exit;
            }
            char * macAddressStringOTA = item->valuestring;
            char macStringOTA [13];
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
                macStringOTA[i] = *(macAddressStringOTA + i + j);
            }
            macStringOTA[12] = '\0'; 
            mlink_mac_str2hex(macStringOTA, mac);
            memcpy(request->addrs_list + i * MWIFI_ADDR_LEN, mac, MWIFI_ADDR_LEN);
            i++;
        }
    }

    else{
        if (value == NULL) {
             MDF_LOGE("DATA IS NULL --> GOTO EXIT");
             MDF_FREE(request->addrs_list);
             MDF_FREE(request);
             goto _exit;
        }
        
        //printf("VALUE: %d\n",atoi(value->valuestring));
        request->value = atoi(value->valuestring);              
        request->addrs_num = 1;
        request->addrs_list = MDF_MALLOC(MWIFI_ADDR_LEN * request->addrs_num);           
        mlink_mac_str2hex(macString,mac);
        memcpy(request->addrs_list, mac, MWIFI_ADDR_LEN);
    }

_exit:
    cJSON_Delete(obj);
    if (request == NULL){
        MDF_LOGE("REQUEST IS NULL");
    }
    else{
        MDF_LOGE("REQUEST IS NOT NULL");
    }
    return request;
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    MDF_LOGE("mqtt event handler");
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            MDF_LOGD("MQTT_EVENT_CONNECTED");
            g_mesh_mqtt.is_connected = true;
            mdf_event_loop_send(MDF_EVENT_CUSTOM_MQTT_CONNECT, NULL);
            break;

        case MQTT_EVENT_DISCONNECTED:
            MDF_LOGD("MQTT_EVENT_DISCONNECTED");
            g_mesh_mqtt.is_connected = false;
            mdf_event_loop_send(MDF_EVENT_CUSTOM_MQTT_DISCONNECT, NULL);
            break;

        case MQTT_EVENT_SUBSCRIBED:
            MDF_LOGD("MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            MDF_LOGD("MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_PUBLISHED:
            MDF_LOGD("MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_DATA: {
            MDF_LOGD("MQTT_EVENT_DATA, topic: %.*s, data: %.*s",
                     event->topic_len, event->topic, event->data_len, event->data);

            mesh_mqtt_data_t *item = mesh_mqtt_parse_data(event->topic, event->topic_len, event->data, event->data_len);

            if (item == NULL) {
                MDF_LOGI("Return value of mesh_mqtt_parse_data is NULL");
                break;
            }

            if (xQueueSend(g_mesh_mqtt.queue, &item, 0) != pdPASS) {
                MDF_LOGD("Send receive queue failed");
                MDF_FREE(item->addrs_list);
                MDF_FREE(item->data);
                MDF_FREE(item);
            }

            break;
        }

        case MQTT_EVENT_ERROR:
            MDF_LOGD("MQTT_EVENT_ERROR");
            break;

        default:
            MDF_LOGD("Other event id:%d", event->event_id);
            break;
    }

    return ESP_OK;
}

bool mesh_mqtt_is_connect()
{
    return g_mesh_mqtt.is_connected;
}

mdf_err_t mesh_mqtt_subscribe()
{
    MDF_LOGE("mesh mqtt subscribe");
    char topic_str[MESH_MQTT_TOPIC_MAX_LEN];
    //char mac_any[] = MWIFI_ADDR_ANY;
    //char mac_root[] = MWIFI_ADDR_ROOT;



    snprintf(topic_str, sizeof(topic_str), subscribe_topic_template);
    int msg_id = esp_mqtt_client_subscribe(g_mesh_mqtt.client, topic_str, 0);
    MDF_ERROR_CHECK(msg_id < 0, MDF_FAIL, "Subscribe failed");

    return MDF_OK;
}

mdf_err_t mesh_mqtt_unsubscribe()
{
    MDF_LOGE("mesh mqtt unsubscribe");
    char topic_str[MESH_MQTT_TOPIC_MAX_LEN];
    snprintf(topic_str, sizeof(topic_str), subscribe_topic_template);
    int msg_id = esp_mqtt_client_unsubscribe(g_mesh_mqtt.client, topic_str);

    if (msg_id > 0) {
        MDF_LOGI("Unsubscribe: %s, msg_id = %d", topic_str, msg_id);
        return MDF_OK;
    } else {
        MDF_LOGI("Unsubscribe: %s failed", topic_str);
        return MDF_FAIL;
    }
}

mdf_err_t mesh_mqtt_update_topo()
{
    MDF_LOGE("mesh mqtt update topo");
    MDF_ERROR_CHECK(g_mesh_mqtt.client == NULL, MDF_ERR_INVALID_STATE, "MQTT client has not started");

    mdf_err_t ret = MDF_FAIL;
    char mac_addr[13];

    int table_size = esp_mesh_get_routing_table_size();
    mesh_addr_t *route_table = MDF_CALLOC(table_size, sizeof(mesh_addr_t));
    ESP_ERROR_CHECK(esp_mesh_get_routing_table(route_table, table_size * sizeof(mesh_addr_t), &table_size));

    cJSON *obj = cJSON_CreateArray();

    for (int i = 0; i < table_size; i++) {
        mlink_mac_hex2str(route_table[i].addr, mac_addr);
        cJSON *item = cJSON_CreateString(mac_addr);
        MDF_ERROR_GOTO(item == NULL, _no_mem, "Create string object failed");
        cJSON_AddItemToArray(obj, item);
    }

    MDF_FREE(route_table);
    char *str = cJSON_PrintUnformatted(obj);
    MDF_ERROR_GOTO(str == NULL, _no_mem, "Print JSON failed");
    esp_mqtt_client_publish(g_mesh_mqtt.client, g_mesh_mqtt.topo_topic, str, strlen(str), 0, 0);
    MDF_FREE(str);
    ret = MDF_OK;
_no_mem:
    cJSON_Delete(obj);
    return ret;
}

mdf_err_t mesh_mqtt_write(uint8_t *addr, const char *data, size_t size, mesh_mqtt_publish_data_type_t type)
{
    
    char macString [18];
    mdf_err_t ret = MDF_FAIL;
    MDF_LOGE("mesh mqtt write");
    MDF_PARAM_CHECK(addr);
    MDF_PARAM_CHECK(data);
    MDF_ERROR_CHECK(type >= MESH_MQTT_DATA_TYPE_MAX, MDF_ERR_INVALID_ARG, "Unknow data type");
    MDF_ERROR_CHECK(g_mesh_mqtt.client == NULL, MDF_ERR_INVALID_STATE, "MQTT client has not started");
    //printf("TRYING TO SEND MQTT MESSAGE\n");

    cJSON *obj1 = cJSON_Parse(data);
    cJSON *address = cJSON_GetObjectItem(obj1, "Mac Address");
    char * macAddressString = address->valuestring;

    cJSON *channel = cJSON_GetObjectItem(obj1, "CH");

    cJSON *messageValue = cJSON_GetObjectItem(obj1, "VALUE");
    
    int j = 0;
    for(int i = 0; i <18; i++){
        switch(i){
            case 2: macString[2] = ':';j = j + 1; ;break;
            case 5: macString[5] = ':';j = j + 1; ;break;
            case 8: macString[8] = ':';j = j + 1; ;break;
            case 11: macString[11] = ':';j = j + 1; ;break;
            case 14: macString[14] = ':';j = j + 1; ;break;
            case 17: macString[17] = '\0';j = j + 1; ;break;
            default: macString[i] = *(macAddressString + i - j); 
        }       
    }

    cJSON *obj2 = cJSON_CreateObject();
    MDF_ERROR_GOTO(obj2 == NULL, _no_mem, "Create JSON failed");

    cJSON *src_addr = cJSON_AddStringToObject(obj2, "mac", macString);
    MDF_ERROR_GOTO(src_addr == NULL, _no_mem, "Add string to JSON failed");
    
    // cJSON *channel_s = cJSON_AddStringToObject(obj2, "channel", channel->valuestring);
    // MDF_ERROR_GOTO(channel_s == NULL, _no_mem, "Add string to JSON failed");
    cJSON *channel_s = cJSON_AddNumberToObject(obj2, "channel", channel->valueint);
    MDF_ERROR_GOTO(channel_s == NULL, _no_mem, "Add string to JSON failed");

    int len = strlen(messageValue->valuestring);
    char *buffer = MDF_MALLOC(len + 1);
    MDF_ERROR_GOTO(buffer == NULL, _no_mem, "Allocate mem failed");
    memcpy(buffer, messageValue->valuestring, len);
    buffer[len] = '\0';
    cJSON *value_s = cJSON_AddStringToObject(obj2, "value", buffer);
    MDF_FREE(buffer);
    MDF_ERROR_GOTO(value_s == NULL, _no_mem, "Add string to JSON failed");

    char *payload = cJSON_PrintUnformatted(obj2);
    MDF_ERROR_GOTO(payload == NULL, _no_mem, "Print JSON failed");

    esp_mqtt_client_publish(g_mesh_mqtt.client, g_mesh_mqtt.publish_topic, payload, strlen(payload), 0, 0);
    MDF_FREE(payload);
    
    ret = MDF_OK;
_no_mem: 
    cJSON_Delete(obj2);
    cJSON_Delete(obj1);
    
    return ret;
}

mdf_err_t mesh_mqtt_read(mesh_mqtt_data_t **request, TickType_t wait_ticks)
{
    //MDF_LOGE("mesh mqtt read");
    MDF_PARAM_CHECK(request);
    MDF_ERROR_CHECK(g_mesh_mqtt.client == NULL, MDF_ERR_INVALID_STATE, "MQTT client has not started");

    if (xQueueReceive(g_mesh_mqtt.queue, request, wait_ticks) != pdPASS) {
        return MDF_ERR_TIMEOUT;
    }

    return MDF_OK;
}

mdf_err_t mesh_mqtt_start(char *url)
{
    MDF_LOGE("mesh mqtt start");
    MDF_PARAM_CHECK(url);
    MDF_ERROR_CHECK(g_mesh_mqtt.client != NULL, MDF_ERR_INVALID_STATE, "MQTT client is already running");

    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = url,
        .event_handle = mqtt_event_handler,
        // .client_cert_pem = (const char *)client_cert_pem_start,
        // .client_key_pem = (const char *)client_key_pem_start,
    };
    MDF_ERROR_ASSERT(esp_read_mac(g_mesh_mqtt.addr, ESP_MAC_WIFI_STA));
    snprintf(g_mesh_mqtt.publish_topic, sizeof(g_mesh_mqtt.publish_topic), publish_topic_template);
    
    g_mesh_mqtt.queue = xQueueCreate(5, sizeof(mesh_mqtt_data_t *));
    g_mesh_mqtt.client = esp_mqtt_client_init(&mqtt_cfg);
    MDF_ERROR_ASSERT(esp_mqtt_client_start(g_mesh_mqtt.client));

    return MDF_OK;
}

mdf_err_t mesh_mqtt_stop()
{
    MDF_LOGE("mesh mqtt stop");
    MDF_ERROR_CHECK(g_mesh_mqtt.client == NULL, MDF_ERR_INVALID_STATE, "MQTT client has not been started");
    mesh_mqtt_data_t *item;

    if (uxQueueMessagesWaiting(g_mesh_mqtt.queue)) {
        if (xQueueReceive(g_mesh_mqtt.queue, &item, 0)) {
            MDF_FREE(item);
        }
    }

    vQueueDelete(g_mesh_mqtt.queue);
    g_mesh_mqtt.queue = NULL;

    esp_mqtt_client_stop(g_mesh_mqtt.client);
    esp_mqtt_client_destroy(g_mesh_mqtt.client);
    g_mesh_mqtt.client = NULL;

    return MDF_OK;
}

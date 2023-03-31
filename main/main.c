/**
 * @file main.c
 * @author Annim Banerjee (pixma38@gmail.com)
 * @brief NA
 * @version 0.1
 * @date 2023-03-06
 *
 * @copyright NA
 *
 */

// main header goes here
#include "./inc/main.h"

// BLE include starts here
#include "./inc/ble_activity.h"
// End of BLE includes

// WiFi related includes starts here
#include "./inc/wifi_activity.h"
// WiFi related includes ends here.

// MQTT Related header files goes here
#include "mqtt_client.h"
// MQTT related header files ends here

// SPIFFS related includes goes here
#include "esp_spiffs.h"
// SPIFFS related includes ends here

// WiFi related Macros/Definations ends here
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "BLE-WIFI-MQTT-NODE";
static int s_retry_num = 0;

#ifdef SUPPORT_HEARTBEAT
#define ESP_GATT_UUID_SPP_HEARTBEAT 0xABF5
#endif

#ifdef SUPPORT_HEARTBEAT
static QueueHandle_t cmd_heartbeat_queue = NULL;
static uint8_t heartbeat_s[9] = {'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};
static bool enable_heart_ntf = false;
static uint8_t heartbeat_count_num = 0;
#endif

#ifdef SUPPORT_HEARTBEAT
/// SPP Server - Heart beat characteristic, notify&write&read
static const uint16_t spp_heart_beat_uuid = ESP_GATT_UUID_SPP_HEARTBEAT;
static const uint8_t spp_heart_beat_val[2] = {0x00, 0x00};
static const uint8_t spp_heart_beat_ccc[2] = {0x00, 0x00};
#endif


static bool enable_data_ntf = false;
static bool is_connected = false;

// MQTT Related Code starts here
esp_mqtt_client_handle_t client = NULL;
volatile unsigned char ucIsWiFiConnected = false;
volatile unsigned char ucIsMqttConnected = false;
volatile unsigned char ucIsDeviceConnectedViaBluetooth = false;

#define SPIFFS_FILE_PATH "/spiffs/cred.code"
#define STA_SHORT_BUFFER 36

enum
{
    DONE = 1,
    AP_MODE = 10,
    STA_MODE = 11
};
// SPIFFS related code
esp_vfs_spiffs_conf_t conf = {
    .base_path = "/spiffs",
    .partition_label = NULL,
    .max_files = 5,
    .format_if_mount_failed = true};

// 1: index.html for / url
// 2: dashboard.html for /dashboard url
int htmlFilePaths[] = {1, 2, NO_HTML};
// 1: siimple.min.css
// 2: siimple-icons.css
int cssFilePaths[] = {1, 2, NO_CSS};

/* An HTTP server config struct */
static httpd_config_t httpd_config = HTTPD_DEFAULT_CONFIG();

static unsigned char uc_admin_panel_portal_user_logged_in = 0;

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
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

        /*
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        */

        msg_id = esp_mqtt_client_subscribe(client, CONFIG_BROKER_TOPIC_TO_SUBSCRIBE, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        // msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        // ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        // msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };
#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0)
    {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128)
        {
            int c = fgetc(stdin);
            if (c == '\n')
            {
                line[count] = '\0';
                break;
            }
            else if (c > 0 && c < 127)
            {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.broker.address.uri = line;
        printf("Broker url: %s\n", line);
    }
    else
    {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}
// MQTT Related code ends here

// BLE Related Macros/Definations ends here
static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for (int i = 0; i < SPP_IDX_NB; i++)
    {
        if (handle == spp_handle_table[i])
        {
            return i;
        }
    }

    return error;
}

static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
{
    temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));

    if (temp_spp_recv_data_node_p1 == NULL)
    {
        ESP_LOGI(GATTS_TABLE_TAG, "malloc error %s %d\n", __func__, __LINE__);
        return false;
    }
    if (temp_spp_recv_data_node_p2 != NULL)
    {
        temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
    }
    temp_spp_recv_data_node_p1->len = p_data->write.len;
    SppRecvDataBuff.buff_size += p_data->write.len;
    temp_spp_recv_data_node_p1->next_node = NULL;
    temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
    temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
    memcpy(temp_spp_recv_data_node_p1->node_buff, p_data->write.value, p_data->write.len);
    if (SppRecvDataBuff.node_num == 0)
    {
        SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
        SppRecvDataBuff.node_num++;
    }
    else
    {
        SppRecvDataBuff.node_num++;
    }

    return true;
}

static void free_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while (temp_spp_recv_data_node_p1 != NULL)
    {
        temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
        free(temp_spp_recv_data_node_p1->node_buff);
        free(temp_spp_recv_data_node_p1);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
    }

    SppRecvDataBuff.node_num = 0;
    SppRecvDataBuff.buff_size = 0;
    SppRecvDataBuff.first_node = NULL;
}

static void print_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while (temp_spp_recv_data_node_p1 != NULL)
    {
        uart_write_bytes(UART_NUM_0, (char *)(temp_spp_recv_data_node_p1->node_buff), temp_spp_recv_data_node_p1->len);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
    }
}

void uart_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t total_num = 0;
    uint8_t current_num = 0;

    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(spp_uart_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            switch (event.type)
            {
            // Event of UART receving data
            case UART_DATA:
                if ((event.size) && (is_connected))
                {
                    uint8_t *temp = NULL;
                    uint8_t *ntf_value_p = NULL;
#ifdef SUPPORT_HEARTBEAT
                    if (!enable_heart_ntf)
                    {
                        ESP_LOGE(GATTS_TABLE_TAG, "%s do not enable heartbeat Notify\n", __func__);
                        break;
                    }
#endif
                    if (!enable_data_ntf)
                    {
                        ESP_LOGE(GATTS_TABLE_TAG, "%s do not enable data Notify\n", __func__);
                        break;
                    }
                    temp = (uint8_t *)malloc(sizeof(uint8_t) * event.size);
                    if (temp == NULL)
                    {
                        ESP_LOGE(GATTS_TABLE_TAG, "%s malloc.1 failed\n", __func__);
                        break;
                    }
                    memset(temp, 0x0, event.size);
                    uart_read_bytes(UART_NUM_0, temp, event.size, portMAX_DELAY);
                    if (event.size <= (spp_mtu_size - 3))
                    {
                        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], event.size, temp, false);
                    }
                    else if (event.size > (spp_mtu_size - 3))
                    {
                        if ((event.size % (spp_mtu_size - 7)) == 0)
                        {
                            total_num = event.size / (spp_mtu_size - 7);
                        }
                        else
                        {
                            total_num = event.size / (spp_mtu_size - 7) + 1;
                        }
                        current_num = 1;
                        ntf_value_p = (uint8_t *)malloc((spp_mtu_size - 3) * sizeof(uint8_t));
                        if (ntf_value_p == NULL)
                        {
                            ESP_LOGE(GATTS_TABLE_TAG, "%s malloc.2 failed\n", __func__);
                            free(temp);
                            break;
                        }
                        while (current_num <= total_num)
                        {
                            if (current_num < total_num)
                            {
                                ntf_value_p[0] = '#';
                                ntf_value_p[1] = '#';
                                ntf_value_p[2] = total_num;
                                ntf_value_p[3] = current_num;
                                memcpy(ntf_value_p + 4, temp + (current_num - 1) * (spp_mtu_size - 7), (spp_mtu_size - 7));
                                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], (spp_mtu_size - 3), ntf_value_p, false);
                            }
                            else if (current_num == total_num)
                            {
                                ntf_value_p[0] = '#';
                                ntf_value_p[1] = '#';
                                ntf_value_p[2] = total_num;
                                ntf_value_p[3] = current_num;
                                memcpy(ntf_value_p + 4, temp + (current_num - 1) * (spp_mtu_size - 7), (event.size - (current_num - 1) * (spp_mtu_size - 7)));
                                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], (event.size - (current_num - 1) * (spp_mtu_size - 7) + 4), ntf_value_p, false);
                            }
                            vTaskDelay(20 / portTICK_PERIOD_MS);
                            current_num++;
                        }
                        free(ntf_value_p);
                    }
                    free(temp);
                }
                break;
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

static void spp_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 8192, 10, &spp_uart_queue, 0);
    // Set UART parameters
    uart_param_config(UART_NUM_0, &uart_config);
    // Set UART pins
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(uart_task, "uTask", 2048, (void *)UART_NUM_0, 8, NULL);
}

#ifdef SUPPORT_HEARTBEAT
void spp_heartbeat_task(void *arg)
{
    uint16_t cmd_id;

    for (;;)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if (xQueueReceive(cmd_heartbeat_queue, &cmd_id, portMAX_DELAY))
        {
            while (1)
            {
                heartbeat_count_num++;
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                if ((heartbeat_count_num > 3) && (is_connected))
                {
                    esp_ble_gap_disconnect(spp_remote_bda);
                }
                if (is_connected && enable_heart_ntf)
                {
                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_HEARTBEAT_VAL], sizeof(heartbeat_s), heartbeat_s, false);
                }
                else if (!is_connected)
                {
                    break;
                }
            }
        }
    }
    vTaskDelete(NULL);
}
#endif

void spp_cmd_task(void *arg)
{
    uint8_t *cmd_id;

    for (;;)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if (xQueueReceive(cmd_cmd_queue, &cmd_id, portMAX_DELAY))
        {
            esp_log_buffer_char(GATTS_TABLE_TAG, (char *)(cmd_id), strlen((char *)cmd_id));
            free(cmd_id);
        }
    }
    vTaskDelete(NULL);
}

static void spp_task_init(void)
{
    spp_uart_init();

#ifdef SUPPORT_HEARTBEAT
    cmd_heartbeat_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_heartbeat_task, "spp_heartbeat_task", 2048, NULL, 10, NULL);
#endif

    cmd_cmd_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_cmd_task, "spp_cmd_task", 2048, NULL, 10, NULL);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    ESP_LOGE(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // advertising start complete event to indicate advertising start successfully or failed
        if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s\n", esp_err_to_name(err));
        }
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *)param;
    uint8_t res = 0xff;

    ESP_LOGI(GATTS_TABLE_TAG, "event = %x\n", event);
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);

        ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));

        ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
        break;
    case ESP_GATTS_READ_EVT:
        res = find_char_and_desr_index(p_data->read.handle);
        if (res == SPP_IDX_SPP_STATUS_VAL)
        {
            // TODO:client read the status characteristic
        }
        break;
    case ESP_GATTS_WRITE_EVT:
    {
        res = find_char_and_desr_index(p_data->write.handle);
        if (p_data->write.is_prep == false)
        {
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d\n", res);
            if (res == SPP_IDX_SPP_COMMAND_VAL)
            {
                uint8_t *spp_cmd_buff = NULL;
                spp_cmd_buff = (uint8_t *)malloc((spp_mtu_size - 3) * sizeof(uint8_t));
                if (spp_cmd_buff == NULL)
                {
                    ESP_LOGE(GATTS_TABLE_TAG, "%s malloc failed\n", __func__);
                    break;
                }
                memset(spp_cmd_buff, 0x0, (spp_mtu_size - 3));
                memcpy(spp_cmd_buff, p_data->write.value, p_data->write.len);
                xQueueSend(cmd_cmd_queue, &spp_cmd_buff, 10 / portTICK_PERIOD_MS);
            }
            else if (res == SPP_IDX_SPP_DATA_NTF_CFG)
            {
                if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x01) && (p_data->write.value[1] == 0x00))
                {
                    enable_data_ntf = true;
                }
                else if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x00) && (p_data->write.value[1] == 0x00))
                {
                    enable_data_ntf = false;
                }
            }
#ifdef SUPPORT_HEARTBEAT
            else if (res == SPP_IDX_SPP_HEARTBEAT_CFG)
            {
                if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x01) && (p_data->write.value[1] == 0x00))
                {
                    enable_heart_ntf = true;
                }
                else if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x00) && (p_data->write.value[1] == 0x00))
                {
                    enable_heart_ntf = false;
                }
            }
            else if (res == SPP_IDX_SPP_HEARTBEAT_VAL)
            {
                if ((p_data->write.len == sizeof(heartbeat_s)) && (memcmp(heartbeat_s, p_data->write.value, sizeof(heartbeat_s)) == 0))
                {
                    heartbeat_count_num = 0;
                }
            }
#endif
            else if (res == SPP_IDX_SPP_DATA_RECV_VAL)
            {
#ifdef SPP_DEBUG_MODE
                esp_log_buffer_char(GATTS_TABLE_TAG, (char *)(p_data->write.value), p_data->write.len);
#else
                uart_write_bytes(UART_NUM_0, (char *)(p_data->write.value), p_data->write.len);
                // This is where you will find you incoming data over BLE in:
                // p_data->write.value with length in p_data->write.len.
#endif
            }
            else
            {
                // TODO:
            }
        }
        else if ((p_data->write.is_prep == true) && (res == SPP_IDX_SPP_DATA_RECV_VAL))
        {
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d\n", res);
            store_wr_buffer(p_data);
        }
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT\n");
        if (p_data->exec_write.exec_write_flag)
        {
            print_write_buffer();
            free_write_buffer();
        }
        break;
    }
    case ESP_GATTS_MTU_EVT:
        spp_mtu_size = p_data->mtu.mtu;
        break;
    case ESP_GATTS_CONF_EVT:
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        spp_conn_id = p_data->connect.conn_id;
        spp_gatts_if = gatts_if;
        is_connected = true;
        memcpy(&spp_remote_bda, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
#ifdef SUPPORT_HEARTBEAT
        uint16_t cmd = 0;
        xQueueSend(cmd_heartbeat_queue, &cmd, 10 / portTICK_PERIOD_MS);
#endif
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        is_connected = false;
        enable_data_ntf = false;
#ifdef SUPPORT_HEARTBEAT
        enable_heart_ntf = false;
        heartbeat_count_num = 0;
#endif
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GATTS_OPEN_EVT:
        break;
    case ESP_GATTS_CANCEL_OPEN_EVT:
        break;
    case ESP_GATTS_CLOSE_EVT:
        break;
    case ESP_GATTS_LISTEN_EVT:
        break;
    case ESP_GATTS_CONGEST_EVT:
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x\n", param->add_attr_tab.num_handle);
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != SPP_IDX_NB)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
        }
        else
        {
            memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
            esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
        }
        break;
    }
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d\n", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n", param->reg.app_id, param->reg.status);
            return;
        }
    }

    do
    {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == spp_profile_tab[idx].gatts_if)
            {
                if (spp_profile_tab[idx].gatts_cb)
                {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

// WiFi related code goes here
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < DEVICE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
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

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = DEVICE_ESP_WIFI_SSID,
            .password = DEVICE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

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
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 DEVICE_ESP_WIFI_SSID, DEVICE_ESP_WIFI_PASS);

        // MQTT Task Init/Call here
        mqtt_app_start();
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 DEVICE_ESP_WIFI_SSID, DEVICE_ESP_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static esp_err_t siimple_css_get_handler(httpd_req_t *req)
{
    // const char* file_path = req->uri;
    int file_fd = open(FILE_PATH_SIIMPLE_CSS, O_RDONLY, 0);
    if (file_fd == -1)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    struct stat file_stat;
    if (fstat(file_fd, &file_stat) == -1)
    {
        ESP_LOGE(TAG, "Failed to stat file");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/css");
    // httpd_resp_set_hdr(req, "Content-Encoding", "gzip");

    char *chunk = (char *)malloc(file_stat.st_size);
    if (!chunk)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for file");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int read_len = read(file_fd, chunk, file_stat.st_size);
    if (read_len == -1)
    {
        ESP_LOGE(TAG, "Failed to read file");
        httpd_resp_send_500(req);
        free(chunk);
        return ESP_FAIL;
    }

    httpd_resp_send(req, chunk, read_len);
    free(chunk);
    close(file_fd);
    return ESP_OK;
}

static esp_err_t contentProvider(httpd_req_t *req, enum httpRequestedFileType fileType, int htmlFilePaths, int cssFilePaths)
{
    char *buffer = malloc(BUFFER_LIMIT * sizeof(char));
    char *content = malloc(ESP_VFS_PATH_MAX * sizeof(char)); // allocate memory for content
    char *filePath = NULL;
    size_t bytes_sent = 0;
    size_t bytes_read = 0;

    if (fileType == HTML_FILE)
    {
        /* Send file contents as HTTP response using buffer limit */
        httpd_resp_set_type(req, "text/html");
        switch (htmlFilePaths)
        {
        case NO_HTML:
            content = "No HTML content available";
            break;
        case 1:
            filePath = FILE_PATH_INDEX_HTML;
            break;
        case 2:
            filePath = FILE_PATH_DASHBOARD_HTML;
            break;
        default:
            content = "Invalid HTML file path";
            break;
        }
    }
    else if (fileType == CSS_FILE)
    {
        httpd_resp_set_type(req, "text/css");
        // httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        switch (cssFilePaths)
        {
        case NO_CSS:
            content = "No CSS content available";
            break;
        case 1:
            filePath = FILE_PATH_SIIMPLE_CSS;
            break;
        case 2:
            filePath = FILE_PATH_SIIMPLE_CSS_ICONS;
            break;
        default:
            content = "Invalid CSS file path";
            break;
        }
    }
    else
    {
        content = "Invalid file type requested";
    }

    if (filePath != NULL)
    {
        /* Open file for reading */
        FILE *fp = fopen(filePath, "r");
        if (!fp)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open file");
            ESP_LOGE(TAG, "%s: HTTPD_500_INTERNAL_SERVER_ERROR. Failed to open file\n", __func__);
            return ESP_FAIL;
        }

        while ((bytes_read = fread(buffer, 1, BUFFER_LIMIT, fp)) > 0)
        {
            // ESP_LOGI(TAG, "%s:%s", __func__, buffer);
            httpd_resp_send_chunk(req, buffer, bytes_read);
            bytes_sent += bytes_read;
            if (bytes_sent >= SPIFFS_MAX_FILE_SIZE)
            {
                ESP_LOGI(TAG, "%s:%s", __func__, "index.html is larger than SPIFFS_MAX_FILE_SIZE. Breaking this loop.");
                break;
            }
        }
        fclose(fp);

        /* End the HTTP response */
        httpd_resp_send_chunk(req, NULL, 0);
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

/* An HTTP GET handler for the root URL */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    // request , fileType, File Index, CSS index.
    return contentProvider(req, HTML_FILE, INDEX_HTML_PATH, INDEX_CSS_PATH) ;
    
    
    // char buffer[BUFFER_LIMIT];

    // /* Open file for reading */
    // FILE *fp = fopen(FILE_PATH_INDEX_HTML, "r");
    // if (!fp)
    // {
    //     httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open file");
    //     ESP_LOGE(TAG, "%s: HTTPD_500_INTERNAL_SERVER_ERROR. Failed to open file\n", __func__);
    //     return ESP_FAIL;
    // }

    // /* Send file contents as HTTP response using buffer limit */
    // httpd_resp_set_type(req, "text/html");
    // size_t bytes_sent = 0;
    // size_t bytes_read = 0;
    // while ((bytes_read = fread(buffer, 1, BUFFER_LIMIT, fp)) > 0)
    // {
    //     // ESP_LOGI(TAG, "%s:%s", __func__, buffer);
    //     httpd_resp_send_chunk(req, buffer, bytes_read);
    //     bytes_sent += bytes_read;
    //     if (bytes_sent >= SPIFFS_MAX_FILE_SIZE)
    //     {
    //         ESP_LOGI(TAG, "%s:%s", __func__, "index.html is larger than SPIFFS_MAX_FILE_SIZE. Breaking this loop.");
    //         break;
    //     }
    // }
    // fclose(fp);

    // /* End the HTTP response */
    // httpd_resp_send_chunk(req, NULL, 0);

    // ESP_LOGI(TAG, "%s:%s", __func__, "Done sending index.html...");

    // return ESP_OK;
}
//
static esp_err_t auth_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;
    char username[50], password[50];

    ESP_LOGI(TAG, "%s:Received Content Length: %d.", __func__, remaining);

    // Read the request body (POST data)
    while (remaining > 0)
    {
        ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));
        if (ret <= 0)
        {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
            {
                // Retry receiving if timeout occurred
                continue;
            }
            ESP_LOGE(TAG, "Error receiving POST data");
            return ESP_FAIL;
        }
        remaining -= ret;

        ESP_LOGI(TAG, "%s:Buf Copied out of POST data packet: %s.", __func__, buf);
        // Parse username and password from POST data
        if (remaining >= 0)
        {
            char *ptr = strstr(buf, "username=");
            if (ptr != NULL)
            {
                strncpy(username, ptr + strlen("username="), strlen(ESP_AP_ADMIN_USERNAME)); // sizeof(username) - 1);
                // username[sizeof(username) - 1] = '\0';
                username[strlen(ESP_AP_ADMIN_USERNAME)] = '\0'; // End with null char
                ESP_LOGI(TAG, "%s:From Buf, username extracted: %s.", __func__, username);
            }
            ptr = strstr(buf, "password=");
            if (ptr != NULL)
            {
                strncpy(password, ptr + strlen("password="), strlen(ESP_AP_ADMIN_PASSWORD));
                password[strlen(ESP_AP_ADMIN_PASSWORD)] = '\0'; // End with null char
                ESP_LOGI(TAG, "%s:From Buf, password extracted: %s.", __func__, password);
            }
        }
    }

    // Check if username and password match
    if (strcmp(username, ESP_AP_ADMIN_USERNAME) == 0 && strcmp(password, ESP_AP_ADMIN_PASSWORD) == 0)
    {
        uc_admin_panel_portal_user_logged_in = 1;
        // Send "OK" response
        const char *resp_str = "OK";
        httpd_resp_send(req, resp_str, strlen(resp_str));
    }
    else
    {
        uc_admin_panel_portal_user_logged_in = 0;
        // Send "FAIL" response
        const char *resp_str = "FAIL";
        ESP_LOGI(TAG, "%s:From Buf, password extracted: %s | Not Macthed, hence FAIL.", __func__, password);
        httpd_resp_send(req, resp_str, strlen(resp_str));
        /* End the HTTP response */
        httpd_resp_send_chunk(req, NULL, 0);
        return ESP_FAIL;
    }
    /* End the HTTP response */
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}
//
static esp_err_t dashboard_get_handler(httpd_req_t *req){
    // request , fileType, File Index, CSS index.
    return contentProvider(req, HTML_FILE, DASHBOARD_HTML_PATH, DASHBOARD_CSS_PATH);
}
//
int switchToAccessPointMode()
{
    /* Initialize the TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_init());
    /* Create and initialize the event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Create and initialize the default network interface */
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    /* Configure the WiFi AP */
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = CONFIG_ESP_SOFTAP_WIFI_SSID,
            .ssid_len = strlen(CONFIG_ESP_SOFTAP_WIFI_SSID),
            .channel = CONFIG_ESP_SOFTAP_WIFI_CHANNEL,
            .password = CONFIG_ESP_SOFTAP_WIFI_PASSWORD,
            .max_connection = CONFIG_ESP_SOFTAP_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        }};
    // strncpy((char *) wifi_config.ap.password, CONFIG_ESP_SOFTAP_WIFI_PASSWORD, sizeof(wifi_config.ap.password));

    /* Set the WiFi mode to SoftAP */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    /* Configure the WiFi AP with the given configuration */
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    /* Start the WiFi AP */
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Start the HTTP server */
    // esp-idf for esp32 issue number #3373
    // no wildcard support IDF Version 3.2
    httpd_handle_t server = NULL;
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL};

    httpd_uri_t siimple_css_uri = {
        .uri = "/siimple.min.css", // match any URI that ends with .css
        .method = HTTP_GET,
        .handler = siimple_css_get_handler,
        .user_ctx = NULL};

    httpd_uri_t auth_login = {
        .uri = "/auth", // match any URI that ends with .css
        .method = HTTP_POST,
        .handler = auth_post_handler,
        .user_ctx = NULL};

    httpd_uri_t panel_dashboard = {
        .uri = "/dashboard", // match any URI that ends with .css
        .method = HTTP_GET,
        .handler = dashboard_get_handler,
        .user_ctx = NULL};

    httpd_config.server_port = ESP_AP_SERVER_PORT;
    ESP_ERROR_CHECK(httpd_start(&server, &httpd_config));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &root_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &siimple_css_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &auth_login));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &panel_dashboard));
    ESP_LOGI(TAG, "HTTP server started on port %d", ESP_AP_SERVER_PORT);

    /* Print the WiFi AP SSID and password */
    ESP_LOGI(TAG, "WiFi AP SSID: %s", CONFIG_ESP_SOFTAP_WIFI_SSID);
    ESP_LOGI(TAG, "WiFi AP password: %s", CONFIG_ESP_SOFTAP_WIFI_PASSWORD);

    return 1;
}
int switchToStationMode()
{
    // WiFi related code goes here.
    wifi_init_sta();
    // WiFi related code ends here.

    return 1;
}
// end of int switchToAccessPointMode() function body.

// Creds related code starts here
int whichMode()
{
    char *credsJsonStr;
    ESP_LOGI(TAG, "Checking for avilable Credentials...");

    /*
    "r": Opens a file for reading. The file must exist.
    "w": Opens a file for writing. If the file does not exist, it will be created. If it exists, its contents will be truncated to zero length before writing begins.
    "a": Opens a file for appending. If the file does not exist, it will be created. If it exists, writing will begin at the end of the file.
    "r+": Opens a file for both reading and writing. The file must exist.
    "w+": Opens a file for both reading and writing. If the file does not exist, it will be created. If it exists, its contents will be truncated to zero length before writing begins.
    "a+": Opens a file for both reading and appending. If the file does not exist, it will be created. If it exists, writing will begin at the end of the file.
    "b": Binary mode. This flag is used to open a file in binary mode. If this flag is not specified, the file will be opened in text mode.
    "t": Text mode. This flag is used to open a file in text mode. If this flag is not specified, the file will be opened in binary mode.
    "x": Creates a new file and opens it for writing. If the file already exists, fopen() will fail.
    "e": Fail if the file does not exist. This flag is used with the "r" mode to ensure that the file being opened actually exists. If the file does not exist, fopen() will fail.
    Note that the "x" and "e" flags are specific to the SPIFFS file system and may not be available in other file systems.
    */

    FILE *file = fopen(SPIFFS_FILE_PATH, "r");
    if (file == NULL)
    {
        file = fopen(SPIFFS_FILE_PATH, "w");
        if (file == NULL)
        {
            return 0;
        }
        credsJsonStr = "{}";
        fwrite(credsJsonStr, strlen(credsJsonStr), 1, file);
        fclose(file);
        return AP_MODE;
    }
    else
    {
        // Read JSON string from file
        fseek(file, 0, SEEK_END);
        long fsize = ftell(file);
        fseek(file, 0, SEEK_SET);
        credsJsonStr = (char *)malloc(fsize + 1);
        fread(credsJsonStr, fsize, 1, file);
        fclose(file);
        credsJsonStr[fsize] = '\0';
    }

    // Parse JSON string and retrieve value for ssidName key
    cJSON *credsJson = cJSON_Parse(credsJsonStr);
    if (credsJson == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "%s: Error parsing JSON: %s", __func__, error_ptr);
        }
        free(credsJsonStr);
        return 0;
    }

    cJSON *val;
    if ((val = cJSON_GetObjectItemCaseSensitive(credsJson, "ssidName")) != NULL)
    {
        if (strlen(val->valuestring) == 0)
        {
            cJSON_Delete(credsJson);
            free(credsJsonStr);
            return AP_MODE;
        }
        else
        {
            cJSON_Delete(credsJson);
            free(credsJsonStr);
            return STA_MODE;
        }
    }
    else
    {
        cJSON_Delete(credsJson);
        free(credsJsonStr);
        return 0;
    }
}
// Creds related code ends here

int checkSpiffSystem()
{
    esp_err_t ret;
    size_t total = 0, used = 0;

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        else if (ret == ESP_ERR_INVALID_STATE)
        {
            ESP_LOGE(TAG, "Failed, mabe be it is already mounted or partition is encrypted");
        }
        else if (ret == ESP_ERR_NO_MEM)
        {
            ESP_LOGE(TAG, "Failed,  objects could not be allocated.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        // return 0;
    }

    ESP_LOGI(TAG, "Performing SPIFFS_check().");
    ret = esp_spiffs_check(conf.partition_label);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
        // return 0;
    }
    else
    {
        ESP_LOGI(TAG, "SPIFFS_check() successful");
    }

    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s).", esp_err_to_name(ret));
        // esp_spiffs_format(conf.partition_label);
        // return 0;
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    // Check consistency of reported partiton size info.
    if (used > total)
    {
        ESP_LOGW(TAG, "Number of used bytes cannot be larger than total. Performing SPIFFS_check().");
        ret = esp_spiffs_check(conf.partition_label);
        // Could be also used to mend broken files, to clean unreferenced pages, etc.
        // More info at https://github.com/pellepl/spiffs/wiki/FAQ#powerlosses-contd-when-should-i-run-spiffs_check
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
            return 0;
        }
        else
        {
            ESP_LOGI(TAG, "SPIFFS_check() successful");
            return 1;
        }
    }
    else
    {
        return 1;
    }
}

int saveCred(const char *ssidName, const char *passwd, const char *nodeId, const char *infraId, const char *serverIp)
{
    // Create JSON object with the provided key-value pairs
    cJSON *credsJson = cJSON_CreateObject();
    cJSON_AddStringToObject(credsJson, "ssidName", ssidName);
    cJSON_AddStringToObject(credsJson, "passwd", passwd);
    cJSON_AddStringToObject(credsJson, "nodeId", nodeId);
    cJSON_AddStringToObject(credsJson, "infraId", infraId);
    cJSON_AddStringToObject(credsJson, "serverIp", serverIp);

    // Convert JSON object to string
    char *credsJsonStr = cJSON_Print(credsJson);

    FILE *file = fopen(SPIFFS_FILE_PATH, "w");
    if (file == NULL)
    {
        return 0;
    }
    fprintf(file, "%s", credsJsonStr);
    fclose(file);

    // Free memory allocated for JSON object and string
    cJSON_Delete(credsJson);
    return DONE;
}

int readCred(char *ssidName, char *passwd, char *nodeId, char *infraId, char *serverIp)
{
    FILE *file = fopen(SPIFFS_FILE_PATH, "r");
    if (file == NULL)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s: Failed to open file %s for reading.", __func__, SPIFFS_FILE_PATH);
        return 0;
    }
    fseek(file, 0, SEEK_END);
    long fsize = ftell(file);
    fseek(file, 0, SEEK_SET);
    char *credsJsonStr = (char *)malloc(fsize + 1);
    fread(credsJsonStr, fsize, 1, file);
    fclose(file);
    credsJsonStr[fsize] = '\0';

    // Parse JSON string and retrieve values for specified keys
    cJSON *credsJson = cJSON_Parse(credsJsonStr);
    if (credsJson == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL)
        {
            printf("Error parsing JSON: %s\n", error_ptr);
        }
        free(credsJsonStr);
        return 0;
    }
    cJSON *val;
    if ((val = cJSON_GetObjectItemCaseSensitive(credsJson, "ssidName")) != NULL)
    {
        strcpy(ssidName, val->valuestring);
    }
    if ((val = cJSON_GetObjectItemCaseSensitive(credsJson, "passwd")) != NULL)
    {
        strcpy(passwd, val->valuestring);
    }
    if ((val = cJSON_GetObjectItemCaseSensitive(credsJson, "nodeId")) != NULL)
    {
        strcpy(nodeId, val->valuestring);
    }
    if ((val = cJSON_GetObjectItemCaseSensitive(credsJson, "infraId")) != NULL)
    {
        strcpy(infraId, val->valuestring);
    }
    if ((val = cJSON_GetObjectItemCaseSensitive(credsJson, "serverIp")) != NULL)
    {
        strcpy(serverIp, val->valuestring);
    }

    // Free memory allocated for JSON object and string
    cJSON_Delete(credsJson);

    return DONE;
}

/**
 * @brief This function is suppose to reset the fields and area of flash where any credentials are saved in encrypted form or non encrypted form.
 * This fuction can be tailored as per the credentials are saved.
 * 
 * @return int 1 or ESP_OK, 0 or ESP_FAILURE
 */
int factoryReset()
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK)
    {
        return 0;
    }
    FILE *file = fopen(SPIFFS_FILE_PATH, "w");
    if (file == NULL)
    {
        return 0;
    }
    fprintf(file, "ssidName,passwd,nodeId,infraId,serverIp\n");
    for (int i = 0; i < 5; i++)
    {
        fprintf(file, "0x00");
        if (i != 4)
        {
            fprintf(file, ",");
        }
    }
    fclose(file);
    return DONE;
}

// WiFi related code ends here

/**
 * @brief This is the App's main entry point function which returns with void type.
 * 
 */
void app_main(void)
{
    esp_err_t ret;

    if (!checkSpiffSystem())
    {
        ESP_LOGE(GATTS_TABLE_TAG, "From %s: Failed from checkSpiffSystem() : Halting here now.Reset it externally.\n", __func__);
        while (true);
    }

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_INFO);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_DEBUG);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_DEBUG);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    spp_task_init();

    // Check if we have to launch WiFi in SoftAP or in STA mode, if Router AP Credentials Found in Flash.
    switch (whichMode())
    {
    case AP_MODE:
        ESP_LOGI(TAG, "[APP] Startup in AP_MODE.");
        switchToAccessPointMode();
        break;
    case STA_MODE:
        ESP_LOGI(TAG, "[APP] Startup in STA_MODE.");
        switchToStationMode();
        break;
    case 0:
        ESP_LOGE(GATTS_TABLE_TAG, "From %s: [APP] Startup : Failed to find the Mode. Halting here now.Reset it externally.\n", __func__);
        break;

    default:
        break;
    }

    return;
}

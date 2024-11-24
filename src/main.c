#include <esp_timer.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "mqtt_client.h"
#include "nvs_flash.h"

// * MPU Definitions
#define SCALE_FACTOR_2 16384.0
#define SCALE_FACTOR_4 8192.0
#define SCALE_FACTOR_8 4096.0
#define SCALE_FACTOR_16 2048.0

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define SCL 4
#define SDA 5

// * WiFi Definitions

#define EXAMPLE_ESP_WIFI_SSID "DRAKO_IoT"
#define EXAMPLE_ESP_WIFI_PASS "88831473"
#define EXAMPLE_ESP_MAXIMUM_RETRY 1000  // Define a default retry value
#define BROKER_URL "mqtt://10.0.0.57:1883"
// #define BROKER_URL "mqtt://broker.hivemq.com:1883"

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER "MyESP32"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static EventGroupHandle_t s_wifi_event_group;

static const char* TAG_WIFI = "wifi station";
static const char* TAG_MQTT = "mqtt";
static const char* TAG_MPU = "MPU";
static int s_retry_num = 5;

bool mqtt_connected = false;

static void mqtt_app_start(void);

// * WiFi
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG_WIFI, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGI(TAG_WIFI, "Número máximo de intentos alcanzado.");
        }
        ESP_LOGI(TAG_WIFI, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG_WIFI, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG_WIFI, "connected to ap SSID:%s password:%s", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
        mqtt_app_start();
    }
}

void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta =
            {
                .ssid = EXAMPLE_ESP_WIFI_SSID,
                .password = EXAMPLE_ESP_WIFI_PASS,
                /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
                 * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
                 * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
                 * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
                 */
                .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
                .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
                .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
            },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_WIFI, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_WIFI, "connected to ap SSID:%s password:%s", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG_WIFI, "Failed to connect to SSID:%s, password:%s", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG_WIFI, "UNEXPECTED EVENT");
    }
}

// * MQTT
static void log_error_if_nonzero(const char* message, int error_code) {
    if (error_code != 0) {
        ESP_LOGE(TAG_MQTT, "Last error %s: 0x%x", message, error_code);
    }
}

/**
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
    ESP_LOGD(TAG_MQTT, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_CONNECTED");

            mqtt_connected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DISCONNECTED");
            mqtt_connected = false;
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
            ESP_LOGI(TAG_MQTT, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG_MQTT, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default:
            ESP_LOGI(TAG_MQTT, "Other event id:%d", event->event_id);
            break;
    }
}

esp_mqtt_client_handle_t client;

static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = BROKER_URL,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

// * MPU

i2c_master_bus_config_t bus_config = {
    .sda_io_num = SDA,
    .scl_io_num = SCL,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .flags.enable_internal_pullup = true,
};

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t mpu_handle;
i2c_device_config_t mpu_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x68,  // Dirección del MPU-9250
    .scl_speed_hz = 400000,
};

float scale_factor = SCALE_FACTOR_2;

float accel_x_offset = 0.0f;
float accel_y_offset = 0.0f;
float accel_z_offset = 0.0f;

float gyro_x_offset = 0.0f;
float gyro_y_offset = 0.0f;
float gyro_z_offset = 0.0f;

void mpu_init() {
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mpu_cfg, &mpu_handle));

    ESP_LOGI(TAG_MPU, "MPU9250 inicializado.");
}

void mpu_read_accel(float* accel) {
    uint8_t buf[1] = {ACCEL_XOUT_H};
    uint16_t buffer[6];

    ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu_handle, buf, 1, buffer, 6, -1));

    accel[0] = ((float)((int16_t)((buffer[0] << 8) | buffer[1])) - accel_z_offset) / scale_factor * 9.81;
    accel[1] = ((float)((int16_t)((buffer[2] << 8) | buffer[3])) - accel_z_offset) / scale_factor * 9.81;
    accel[2] = ((float)((int16_t)((buffer[4] << 8) | buffer[5])) - accel_z_offset) / scale_factor * 9.81;
}

void mpu_read_raw_accel(int16_t* accel) {
    uint8_t buf[1] = {ACCEL_XOUT_H};
    uint8_t buffer[6];

    ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu_handle, buf, 1, buffer, 6, -1));

    accel[0] = ((buffer[0] << 8) | buffer[1]);
    accel[1] = ((buffer[2] << 8) | buffer[3]);
    accel[2] = ((buffer[4] << 8) | buffer[5]);
}

void calibrate_accel() {
    int16_t accel[3] = {0};
    ESP_LOGI(TAG_MPU, "Iniciando calibración del acelerómetro");
    int32_t accel_x = 0;
    int32_t accel_y = 0;
    int32_t accel_z = 0;
    for (int i = 0; i < 1000; i++) {
        mpu_read_raw_accel(accel);
        accel_x += accel[0];
        accel_y += accel[1];
        accel_z += accel[2];
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    accel_x_offset = (float)(accel_x) / 1000.0f;
    accel_y_offset = (float)(accel_y) / 1000.0f;
    accel_z_offset = (float)(accel_z) / 1000.0f - 16384.0f;
}

void transmit_data_mqtt() {
    calibrate_accel();
    ESP_LOGI(TAG_MQTT, "Iniciando transmisión de datos MQTT");

    int64_t last_time = esp_timer_get_time();

    ESP_LOGW(TAG_MPU, "offsets: %.2f %.2f %.2f", accel_x_offset, accel_y_offset, accel_z_offset);
    while (1) {
        if (!mqtt_connected) {
            ESP_LOGE(TAG_MQTT, "MQTT no conectado");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            last_time = esp_timer_get_time();
            continue;
        }

        float accel[3] = {0.0f};
        mpu_read_accel(accel);

        char data[100];
        int64_t current_time = esp_timer_get_time();
        sprintf(data, "{\"accel\":{\"value\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},\"unit\":\"m/s2\"},\"time\":%lld}", accel[0], accel[1], accel[2], (current_time - last_time));

        esp_mqtt_client_publish(client, "/topic/qos0", data, 0, 0, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        last_time = current_time;
    }
}

void app_main() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG_WIFI, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    ESP_LOGI(TAG_WIFI, "[APP] Startup..");
    ESP_LOGI(TAG_WIFI, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG_WIFI, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    // ESP_ERROR_CHECK(example_connect());

    mpu_init();
    xTaskCreate(transmit_data_mqtt, "transmit_data_mqtt", 4096, NULL, 5, NULL);
}
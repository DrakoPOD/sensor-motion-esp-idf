#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/i2c_master.h"

i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    // .i2c_port = 6,
    .scl_io_num = 4,
    .sda_io_num = 5,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

i2c_master_bus_handle_t bus_handle;

void i2c_scanner() {
    for (uint8_t i = 0; i < 128; i++) {
        if (i2c_master_probe(bus_handle, i, 1000) == ESP_OK) {
            printf("Found device at address: %d\n", i);
        }
    }

    vTaskDelay(300 / portTICK_PERIOD_MS);
}

void app_main() {
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    xTaskCreate(i2c_scanner, "i2c_scanner", 2048, NULL, 10, NULL);
}
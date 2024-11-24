#include "MPU9250_custom.h"

MPU9250::MPU9250() {};

void MPU9250::init(i2c_master_bus_handle_t* bus_handle, uint8_t addr) {
    this->mpu_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    this->mpu_cfg.device_address = addr;  // Dirección del MPU-9250
    this->mpu_cfg.scl_speed_hz = 400000;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &this->mpu_cfg, &this->mpu_handle));

    this->bus_handle = *bus_handle;
};

void MPU9250::read_accel(float* accel) {
    uint8_t buf[1] = {ACCEL_XOUT_H};
    uint8_t buffer[6];

    ESP_ERROR_CHECK(i2c_master_transmit_receive(this->mpu_handle, buf, 1, buffer, 6, -1));

    accel[0] = (((buffer[0] << 8) | buffer[1]) / scale_factor) * 9.81 - accel_x_offset;
    accel[1] = (((buffer[2] << 8) | buffer[3]) / scale_factor) * 9.81 - accel_x_offset;
    accel[2] = (((buffer[4] << 8) | buffer[5]) / scale_factor) * 9.81 - accel_x_offset;
};

void MPU9250::read_gyro(float* gyro) {
    uint8_t buf[1] = {GYRO_XOUT_H};
    uint8_t buffer[6];

    ESP_ERROR_CHECK(i2c_master_transmit_receive(this->mpu_handle, buf, 1, buffer, 6, -1));

    gyro[0] = ((buffer[0] << 8) | buffer[1]) / scale_factor - gyro_x_offset;
    gyro[1] = ((buffer[2] << 8) | buffer[3]) / scale_factor - gyro_y_offset;
    gyro[2] = ((buffer[4] << 8) | buffer[5]) / scale_factor - gyro_z_offset;
};

void MPU9250::calibrate_gyro() {
    int16_t gyro[3] = {0};
    int16_t gyro_offset[3] = {0};

    for (int i = 0; i < 1000; i++) {
        get_pure_gyro(gyro);
        gyro_offset[0] += gyro[0];
        gyro_offset[1] += gyro[1];
        gyro_offset[2] += gyro[2];
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    gyro_x_offset = gyro_offset[0] / 1000.0;
    gyro_y_offset = gyro_offset[1] / 1000.0;
    gyro_z_offset = gyro_offset[2] / 1000.0;
};

void MPU9250::calibrate_accel() {
    int16_t accel[3] = {0};
    int16_t accel_offset[3] = {0};

    for (int i = 0; i < 1000; i++) {
        get_pure_accel(accel);
        accel_offset[0] += accel[0];
        accel_offset[1] += accel[1];
        accel_offset[2] += accel[2];
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    accel_x_offset = accel_offset[0] / 1000.0;
    accel_y_offset = accel_offset[1] / 1000.0;
    accel_z_offset = accel_offset[2] / 1000.0 + GRAVITY;
};

void MPU9250::get_pure_accel(int16_t* accel) {
    uint8_t buf[1] = {ACCEL_XOUT_H};
    uint8_t buffer[6];

    ESP_ERROR_CHECK(i2c_master_transmit_receive(this->mpu_handle, buf, 1, buffer, 6, -1));

    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];
};

void MPU9250::get_pure_gyro(int16_t* gyro) {
    uint8_t buf[1] = {GYRO_XOUT_H};
    uint8_t buffer[6];

    ESP_ERROR_CHECK(i2c_master_transmit_receive(this->mpu_handle, buf, 1, buffer, 6, -1));

    gyro[0] = (buffer[0] << 8) | buffer[1];
    gyro[1] = (buffer[2] << 8) | buffer[3];
    gyro[2] = (buffer[4] << 8) | buffer[5];
};

void MPU9250::reset() {
    const uint8_t pwr_mgmt_1 = PWR_MGMT_1;
    ESP_ERROR_CHECK(i2c_master_transmit(this->mpu_handle, &pwr_mgmt_1, 0x80, -1));

    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
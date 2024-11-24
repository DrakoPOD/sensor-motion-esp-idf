#ifndef MPU9250_CUSTOM_H
#define MPU9250_CUSTOM_H
#pragma once

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "driver/i2c.h"
#include "driver/i2c_master.h"

#define GRAVITY 9.806f
// Scales factors for
#define SCALE_FACTOR_2 16384.0
#define SCALE_FACTOR_4 8192.0
#define SCALE_FACTOR_8 4096.0
#define SCALE_FACTOR_16 2048.0

#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

#define XG_OFFSET_H 0x13
#define XG_OFFSET_L 0x14
#define YG_OFFSET_H 0x15
#define YG_OFFSET_L 0x16
#define ZG_OFFSET_H 0x17
#define ZG_OFFSET_L 0x18

#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D

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

#define WHO_AM_I 0x75

class MPU9250 {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t mpu_handle;
    i2c_device_config_t mpu_cfg;

    float scale_factor = SCALE_FACTOR_2;

    float accel_x_offset = 0.0f;
    float accel_y_offset = 0.0f;
    float accel_z_offset = 0.0f;

    float gyro_x_offset = 0.0f;
    float gyro_y_offset = 0.0f;
    float gyro_z_offset = 0.0f;

   public:
    MPU9250();
    void init(i2c_master_bus_handle_t* bus_handle, uint8_t addr);
    void read_accel(float* accel);
    void read_gyro(float* gyro);
    void calibrate_accel();
    void reset();
    void calibrate_gyro();

   private:
    void get_pure_accel(int16_t* accel);
    void get_pure_gyro(int16_t* gyro);
};

#endif
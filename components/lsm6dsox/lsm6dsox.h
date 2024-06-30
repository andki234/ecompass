/*
 * lsm6dsox.h
 *
 *  Created on: 28 juni 2024
 *      Author: andki
 */

#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include "driver/i2c.h"
#include "esp_err.h"
#include "sensors.h"

#define LSM6DSOX_ADDR 0x6A // I2C address for LSM6DSOX

// LSM6DSOX Register addresses
#define LSM6DSOX_WHO_AM_I   0x0F    // WHO_AM_I register
#define LSM6DSOX_DRDY_PULSE_CFG 0x0B // DRDY_PULSE_CFG register
#define LSM6DSOX_INT1_CTRL  0x0D    // INT1_CTRL register 
#define LSM6DSOX_INT2_CTRL  0x0E    // INT2_CTRL register 
#define LSM6DSOX_CTRL1_XL   0x10    // Control register 1 for accelerometer
#define LSM6DSOX_CTRL2_G    0x11    // Control register 2 for gyroscope
#define LSM6DSOX_CTRL3_C    0x12    // Control register 3 for gyroscope
#define LSM6DSOX_CTRL4_C    0x13    // Control register 4 for accelerometer
#define LSM6DSOX_CTRL6_C    0x15    // Control register 6 for accelerometer    
#define LSM6DSOX_CTRL7_G    0x16    // Control register 7 for gyroscope
#define LSM6DSOX_CTRL8_XL   0x17    // Control register 8 for accelerometer
#define LSM6DSOX_CTRL9_XL   0x18    // Control register 9 for accelerometer
#define LSM6DSOX_STATUS_REG 0x1E    // Status register
#define LSM6DSOX_MD1_CFG    0x5E    // MD1_CFG register 

#define LSM6DSOX_OUTX_L_A   0x28  // Accelerometer X-axis low byte
#define LSM6DSOX_OUTX_H_A   0x29  // Accelerometer X-axis high byte
#define LSM6DSOX_OUTY_L_A   0x2A  // Accelerometer Y-axis low byte
#define LSM6DSOX_OUTY_H_A   0x2B  // Accelerometer Y-axis high byte
#define LSM6DSOX_OUTZ_L_A   0x2C  // Accelerometer Z-axis low byte
#define LSM6DSOX_OUTZ_H_A   0x2D  // Accelerometer Z-axis high byte

#define LSM6DSOX_OUTX_L_G   0x22  // Gyroscope X-axis low byte
#define LSM6DSOX_OUTX_H_G   0x23  // Gyroscope X-axis high byte
#define LSM6DSOX_OUTY_L_G   0x24  // Gyroscope Y-axis low byte
#define LSM6DSOX_OUTY_H_G   0x25  // Gyroscope Y-axis high byte
#define LSM6DSOX_OUTZ_L_G   0x26  // Gyroscope Z-axis low byte

#define LSM6DSOX_WHO_AM_I_VALUE 0x6C // Expected WHO_AM_I value for LSM6DSOX

esp_err_t lsm6dsox_init(i2c_port_t i2c_num, uint8_t i2c_addr);
esp_err_t lsm6dsox_read_accel(i2c_port_t i2c_num, uint8_t i2c_addr, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z);
esp_err_t lsm6dsox_read_gyro(i2c_port_t i2c_num, uint8_t i2c_addr, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);
esp_err_t lsm6dsox_read_accel_gyro(i2c_port_t i2c_num, uint8_t i2c_addr, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

#endif // LSM6DSOX_H


#ifndef LIS3MDL_H
#define LIS3MDL_H

#include "driver/i2c.h"
#include "esp_err.h"
#include "sensors.h"
#include <inttypes.h>

#define LIS3MDL_ADDR 0x1E // I2C address for LIS3MDL

// LIS3MDL Register addresses
#define LIS3MDL_WHO_AM_I    0x0F // WHO_AM_I register
#define LIS3MDL_CTRL_REG1   0x20 // Control register 1
#define LIS3MDL_CTRL_REG2   0x21 // Control register 2
#define LIS3MDL_CTRL_REG3   0x22 // Control register 3
#define LIS3MDL_CTRL_REG4   0x23 // Control register 4
#define LIS3MDL_CTRL_REG5   0x24 // Control register 5
#define LIS3MDL_STATUS_REG  0x27 // Status register

#define LIS3MDL_OUTX_L      0x28 // X-axis low byte
#define LIS3MDL_OUTX_H      0x29 // X-axis high byte
#define LIS3MDL_OUTY_L      0x2A // Y-axis low byte
#define LIS3MDL_OUTY_H      0x2B // Y-axis high byte
#define LIS3MDL_OUTZ_L      0x2C // Z-axis low byte
#define LIS3MDL_OUTZ_H      0x2D // Z-axis high byte

#define LIS3MDL_INT_CFG     0x30 // Interrupt configuration
#define LIS3MDL_INT_SRC     0x31 // Interrupt source

#define LIS3MDL_WHO_AM_I_VALUE 0x3D // Expected WHO_AM_I value for LIS3MDL

esp_err_t lis3mdl_init(i2c_port_t i2c_num, uint8_t i2c_addr);
esp_err_t lis3mdl_read_magnetometer(i2c_port_t i2c_num, uint8_t i2c_addr, int16_t *mag_x, int16_t *mag_y, int16_t *mag_z);
esp_err_t lis3_mdl_read_interrupt_status(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t *istatus);

#endif // LIS3MDL_H

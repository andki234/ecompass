/**
 * @brief Initializes the LSM6DSOX sensor.
 *
 * This function initializes the LSM6DSOX sensor by configuring the I2C port and address.
 *
 * @param i2c_num The I2C port number to use for communication.
 * @param i2c_addr The I2C address of the LSM6DSOX sensor.
 * @return `ESP_OK` if the initialization is successful, otherwise an error code.
 */

#include "lsm6dsox.h"
#include "esp_log.h"

static const char *TAG = "LSM6DSOX";

esp_err_t lsm6dsox_init(i2c_port_t i2c_num, uint8_t i2c_addr) {
    esp_err_t ret;
    uint8_t who_am_i, read_value;

    // Define the register-value pairs for initialization
    struct {
        uint8_t reg;
        uint8_t val;
    } init_values[] = {
        {LSM6DSOX_CTRL1_XL,  0b01000000},   // Accelerometer: 104 Hz, ±2g
        {LSM6DSOX_CTRL2_G,   0b01000010},   // Gyroscope: 104 Hz, 125 dps
        {LSM6DSOX_CTRL3_C,   0b01000100},   // BDU, Auto-increment
        {LSM6DSOX_CTRL6_C,   0b00000000},   // XL High-performance mode
        {LSM6DSOX_CTRL7_G,   0b00000000},   // G High-performance mode
        {LSM6DSOX_CTRL8_XL,  0b00001001},   // XL additional config
        {LSM6DSOX_INT1_CTRL, 0b00000001},   // INT1 config
        {LSM6DSOX_MD1_CFG,   0b00000001}    // Data-ready interrupts on INT1
    };

    // Check WHO_AM_I register
    ret = i2c_master_write_read_device(i2c_num, i2c_addr, (uint8_t[]){LSM6DSOX_WHO_AM_I}, 1, &who_am_i, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK || who_am_i != LSM6DSOX_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "WHO_AM_I register mismatch or read failure");
        return (ret == ESP_OK) ? ESP_FAIL : ret;
    }

    // Initialize the device registers
    for (size_t i = 0; i < sizeof(init_values) / sizeof(init_values[0]); i++) {
        ret = i2c_master_write_to_device(i2c_num, i2c_addr, (uint8_t[]){init_values[i].reg, init_values[i].val}, 2, 1000 / portTICK_PERIOD_MS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write to register 0x%02x", init_values[i].reg);
            return ret;
        }
        // Read back and verify register value
        ret = i2c_master_write_read_device(i2c_num, i2c_addr, &init_values[i].reg, 1, &read_value, 1, 1000 / portTICK_PERIOD_MS);
        if (ret != ESP_OK || read_value != init_values[i].val) {
            ESP_LOGE(TAG, "Failed to verify register 0x%02x: read 0x%02x, expected 0x%02x", init_values[i].reg, read_value, init_values[i].val);
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "LSM6DSOX initialized and configured successfully");
    return ESP_OK;
}



esp_err_t lsm6dsox_read_accel(i2c_port_t i2c_num, uint8_t i2c_addr, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z) {
    uint8_t data[6];
    esp_err_t ret;

    ret = i2c_master_write_read_device(i2c_num, i2c_addr, (uint8_t[]){LSM6DSOX_OUTX_L_A}, 1, data, 6, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer data");
        return ret;
    }

    *accel_x = (data[1] << 8) | data[0];
    *accel_y = (data[3] << 8) | data[2];
    *accel_z = (data[5] << 8) | data[4];

    return ESP_OK;
}

esp_err_t lsm6dsox_read_gyro(i2c_port_t i2c_num, uint8_t i2c_addr, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    uint8_t data[6];
    esp_err_t ret;

    ret = i2c_master_write_read_device(i2c_num, i2c_addr, (uint8_t[]){LSM6DSOX_OUTX_L_G}, 1, data, 6, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read gyroscope data");
        return ret;
    }

    *gyro_x = (data[1] << 8) | data[0];
    *gyro_y = (data[3] << 8) | data[2];
    *gyro_z = (data[5] << 8) | data[4];

    return ESP_OK;
}

esp_err_t lsm6dsox_read_accel_gyro(i2c_port_t i2c_num, uint8_t i2c_addr, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    uint8_t data[12];
    esp_err_t ret;

    ret = i2c_master_write_read_device(i2c_num, i2c_addr, (uint8_t[]){LSM6DSOX_OUTX_L_G}, 1, data, 12, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer and gyroscope data");
        return ret;
    }
    
    
    *gyro_x = (data[1] << 8) | data[0];
    *gyro_y = (data[3] << 8) | data[2];
    *gyro_z = (data[5] << 8) | data[4];

    *accel_x = (data[7] << 8) | data[6];
    *accel_y = (data[9] << 8) | data[8];
    *accel_z = (data[11] << 8) | data[10];


    return ESP_OK;
}

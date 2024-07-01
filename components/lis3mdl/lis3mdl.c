#include "lis3mdl.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "LIS3MDL";

esp_err_t lis3mdl_init(i2c_port_t i2c_num, uint8_t i2c_addr) {
    uint8_t who_am_i, read_value;
    esp_err_t ret;

    // Check WHO_AM_I register
    ret = i2c_master_write_read_device(i2c_num, i2c_addr, (uint8_t[]){LIS3MDL_WHO_AM_I}, 1, &who_am_i, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK || who_am_i != LIS3MDL_WHO_AM_I_VALUE) {
         ESP_LOGE(TAG, "Unexpected WHO_AM_I value: 0x%02x, expected: 0x%02x", who_am_i, LIS3MDL_WHO_AM_I_VALUE);
        return (ret == ESP_OK) ? ESP_FAIL : ret;
    }

    // Define the register-value pairs for initialization
    struct {
        uint8_t reg;
        uint8_t val;
    } init_values[] = {
        {LIS3MDL_CTRL_REG1, 0b01111100},  // Enable temperature sensor, high-performance mode, ODR = 100 Hz
        {LIS3MDL_CTRL_REG2, 0b00000000},  // Set full-scale to ±4 gauss
        {LIS3MDL_CTRL_REG3, 0b00000000},  // Set to continuous-conversion mode
        {LIS3MDL_CTRL_REG4, 0b00001100},  // Z-axis ultra-high-performance mode
        {LIS3MDL_CTRL_REG5, 0b01000000},   // Block data update for magnetic data
        {LIS3MDL_INT_CFG, 0b11101111}      // Enable interrupts on all axes
    };

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

    ESP_LOGI(TAG, "LIS3MDL initialized and configured successfully");
    return ESP_OK;
}

esp_err_t lis3mdl_read_magnetometer(i2c_port_t i2c_num, uint8_t i2c_addr, int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) {
    uint8_t data[6];
    esp_err_t ret;

    ret = i2c_master_write_read_device(i2c_num, i2c_addr, (uint8_t[]){LIS3MDL_OUTX_L}, 1, data, 6, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read magnetometer data");
        return ret;
    }

    *mag_x = (int16_t)(data[1] << 8 | data[0]);
    *mag_y = (int16_t)(data[3] << 8 | data[2]);
    *mag_z = (int16_t)(data[5] << 8 | data[4]);

    return ESP_OK;
}

esp_err_t lis3_mdl_read_interrupt_status(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t *istatus) {
    uint8_t data[1];
    esp_err_t ret;

    ret = i2c_master_write_read_device(i2c_num, i2c_addr, (uint8_t[]){LIS3MDL_INT_SRC}, 1, data, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read interrupt status");
        return ret;
    }

    *istatus = data[0];

    ESP_LOGI(TAG, "Interrupt status: %d", *istatus);
    return ESP_OK;
}
    

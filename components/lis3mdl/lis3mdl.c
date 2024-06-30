#include "lis3mdl.h"
#include "esp_log.h"

static const char *TAG = "LIS3MDL";

esp_err_t lis3mdl_init(i2c_port_t i2c_num, uint8_t i2c_addr) {
    uint8_t who_am_i;
    esp_err_t ret;

    // Check WHO_AM_I register
    ret = i2c_master_write_read_device(i2c_num, i2c_addr, (uint8_t[]){LIS3MDL_WHO_AM_I}, 1, &who_am_i, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }
    if (who_am_i != LIS3MDL_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "Unexpected WHO_AM_I value: 0x%02x, expected: 0x%02x", who_am_i, LIS3MDL_WHO_AM_I_VALUE);
        return ESP_FAIL;
    }

    // Configure control registers
    uint8_t ctrl_reg1_value = 0b01110000;  // Temp sensor disabled, ultra-high performance mode, ODR = 80 Hz
    ret = i2c_master_write_to_device(i2c_num, i2c_addr, (uint8_t[]){LIS3MDL_CTRL_REG1, ctrl_reg1_value}, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL_REG1");
        return ret;
    }

    uint8_t ctrl_reg2_value = 0b00000000;  // Full-scale selection = ±4 gauss
    ret = i2c_master_write_to_device(i2c_num, i2c_addr, (uint8_t[]){LIS3MDL_CTRL_REG2, ctrl_reg2_value}, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL_REG2");
        return ret;
    }

    uint8_t ctrl_reg3_value = 0b00000000;  // Continuous-conversion mode
    ret = i2c_master_write_to_device(i2c_num, i2c_addr, (uint8_t[]){LIS3MDL_CTRL_REG3, ctrl_reg3_value}, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL_REG3");
        return ret;
    }

    uint8_t ctrl_reg4_value = 0b00000000;  // Ultra-high performance mode for Z axis
    ret = i2c_master_write_to_device(i2c_num, i2c_addr, (uint8_t[]){LIS3MDL_CTRL_REG4, ctrl_reg4_value}, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL_REG4");
        return ret;
    }

    uint8_t ctrl_reg5_value = 0b00000000;  // Fast read disabled, block data update enabled
    ret = i2c_master_write_to_device(i2c_num, i2c_addr, (uint8_t[]){LIS3MDL_CTRL_REG5, ctrl_reg5_value}, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL_REG5");
        return ret;
    }

    ESP_LOGI(TAG, "LIS3MDL initialized successfully");
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

    ESP_LOGI(TAG, "Magnetometer data - X: %d, Y: %d, Z: %d", *mag_x, *mag_y, *mag_z);
    return ESP_OK;
}

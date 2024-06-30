#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lsm6dsox.h"
#include "lis3mdl.h"
#include "sensors.h"
#include "uart_wrapper.h"
#include "esp_timer.h"

/*
ESP32S3-DevKitC          LSM6DSOX
+----------------+       +---------+
|                |       |         |
| GPIO 9 (SCL)   +-------+ SCL     |
| GPIO 8 (SDA)   +-------+ SDA     |
| GND            +-------+ GND     |
| 3.3V           +-------+ VCC     |
| GPIO 4 (INT1)  +-------+ INT1    |
+----------------+       +---------+
*/

#define I2C_MASTER_SCL_IO           9           // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO           8           // GPIO number for I2C master data
#define I2C_MASTER_NUM              I2C_NUM_0   // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ          100000      // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0           // I2C master does not need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0           // I2C master does not need buffer
#define I2C_DMA_BUF_SIZE            1024        // Size of the DMA buffer

#define LSM6DSOX_WHO_AM_I_VALUE     0x6C        // Expected WHO_AM_I value for LSM6DSOX
#define LIS3MDL_WHO_AM_I_VALUE      0x3D        // Expected WHO_AM_I value for LIS3MDL

#define LSM6SDOX_GPIO_INT_PIN       0x04        // GPIO pin connected to pin 4 for LSM6DSOX
#define LIS3MDL_GPIO_INT_PIN        0x05        // GPIO pin connected to pin 5 for LIS3MDL

typedef struct {
    uint8_t address;
    sensor_type_t type;
} sensor_info_t;

#define MAX_SENSORS 10

static const char *TAG = "ecompass";
static sensor_info_t sensors[MAX_SENSORS];
static int sensor_count = 0;
static uint8_t sensor_address = 0; // Declare sensor_address as a global variable
static TaskHandle_t sensor_task_handle = NULL;

// Function to initialize the I2C master interface with DMA
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_DMA_BUF_SIZE, I2C_DMA_BUF_SIZE, 0);
}

// Function to scan I2C bus for devices and return the list of sensors found
static esp_err_t i2c_scan_for_sensors(sensor_info_t *sensors, int *sensor_count) {
    int i;
    esp_err_t espRc;
    *sensor_count = 0;

    for (i = 1; i < 126; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        espRc = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (espRc == ESP_OK) {
            // Check if the device is the LSM6DSOX or LIS3MDL by reading the WHO_AM_I register
            uint8_t who_am_i;
            espRc = i2c_master_write_read_device(I2C_MASTER_NUM, i, (uint8_t[]){LSM6DSOX_WHO_AM_I}, 1, &who_am_i, 1, 1000 / portTICK_PERIOD_MS);
            if (espRc == ESP_OK) {
                sensor_type_t type = SENSOR_UNKNOWN;
                if (who_am_i == LSM6DSOX_WHO_AM_I_VALUE) {
                    type = SENSOR_LSM6DSOX;
                    ESP_LOGI(TAG, "Found LSM6DSOX device at address 0x%02x", i);
                } else if (who_am_i == LIS3MDL_WHO_AM_I_VALUE) {
                    type = SENSOR_LIS3MDL;
                    ESP_LOGI(TAG, "Found LIS3MDL device at address 0x%02x", i);
                }

                sensors[*sensor_count].address = i;
                sensors[*sensor_count].type = type;
                (*sensor_count)++;

                if (*sensor_count >= MAX_SENSORS) {
                    break;
                }
            }
        }
    }
    return (*sensor_count > 0) ? ESP_OK : ESP_FAIL;
}


// Interrupt variables for LSM6DSOX INT1 pin
static int64_t _last_interrupt_time_int1 = 0;
static int64_t _current_time_int1 = 0;
static int64_t _duration_int1 = 0;

// Interrupt variables for LIS3MDL INT2 pin
static int64_t _last_interrupt_time_int2 = 0;
static int64_t _current_time_int2 = 0;
static int64_t _duration_int2 = 0;

// ISR handler function
void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    // Handle the interrupt (e.g., toggle an LED or set a flag)
    if (gpio_num == LSM6SDOX_GPIO_INT_PIN) {
        // Handle interrupt for LSM6SDOX_GPIO_INT_PIN (pin 4)
        _current_time_int1 = esp_timer_get_time();
        _duration_int1 = _current_time_int1 - _last_interrupt_time_int1;
        _last_interrupt_time_int1 = _current_time_int1;
        xTaskNotifyFromISR(sensor_task_handle, gpio_num, eSetBits, NULL);
    } else if (gpio_num == LIS3MDL_GPIO_INT_PIN) {
        // Handle interrupt for LIS3MDL_GPIO_INT_PIN (pin 5)
        _current_time_int2 = esp_timer_get_time();
        _duration_int2 = _current_time_int2 - _last_interrupt_time_int2;
        _last_interrupt_time_int2 = _current_time_int2;
    }
}


/**
 * Configures the GPIO interrupts.
 *
 * This function is responsible for configuring the GPIO interrupts for the LSM6DSOX and LIS3MDL sensors.
 * 
 * @param None
 * @return None
 */
void configure_gpio_interrupts() {
    // Configure the GPIO pins
    gpio_config_t io_conf;

    // Configure pin 4
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << LSM6SDOX_GPIO_INT_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Configure pin 5
    io_conf.pin_bit_mask = (1ULL << LIS3MDL_GPIO_INT_PIN);
    gpio_config(&io_conf);

    // Install GPIO ISR service
    gpio_install_isr_service(0);

    // Hook ISR handler for the specific pins
    gpio_isr_handler_add(LSM6SDOX_GPIO_INT_PIN, gpio_isr_handler, (void*) LSM6SDOX_GPIO_INT_PIN);  
    gpio_isr_handler_add(LIS3MDL_GPIO_INT_PIN, gpio_isr_handler, (void*) LIS3MDL_GPIO_INT_PIN);    
}

static int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;

void sensor_task(void* arg) {
    uint32_t io_num;
    uint8_t status;
    esp_err_t ret;
    
    while (1) {
        if (xTaskNotifyWait(0x00, ULONG_MAX, &io_num, portMAX_DELAY)) {
            ret = i2c_master_write_read_device(I2C_MASTER_NUM, sensor_address, (uint8_t[]){LSM6DSOX_STATUS_REG}, 1, &status, 1, 1000 / portTICK_PERIOD_MS);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read LSM6DSOX status register");
            }
            else if (status &  0b00000011) {
                if (lsm6dsox_read_accel_gyro(I2C_MASTER_NUM, sensor_address, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z) == ESP_OK) {
                    double sample_rate = 1.0 / ((double)_duration_int1 * 1e-6);
                    printf("Time: %lld us | Sample rate: %.2f Hz | Accel: X=%d, Y=%d, Z=%d | Gyro: X=%d, Y=%d, Z=%d\n", _duration_int1, sample_rate, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
                } else {
                    ESP_LOGE(TAG, "Failed to read accelerometer and gyroscope data");
                }
            }
            else {
                ESP_LOGI(TAG, "No data available");
            } 
        }
    }
}


/**
 * @brief Macro for checking and handling custom ESP errors.
 *
 * This macro is used to check the return value of ESP functions and handle any errors that occur.
 * It is typically used in conjunction with ESP functions that return an error code.
 * It is not rebboting as the ESP_ERROR_CHECK macro does.
 * The macro takes a single argument, which is the ESP function call to be checked.
 * If the return value is not ESP_OK, the macro will execute the error handling code.
 * 
 * @param x The ESP function call to be checked.
 */
#define ESP_ERROR_CHECK_CUSTOM(x) do {                                    \
        esp_err_t err_rc_ = (x);                                          \
        if (err_rc_ != ESP_OK) {                                          \
            ESP_LOGE(TAG, "ESP_ERROR_CHECK failed: esp_err_t 0x%x", err_rc_); \
            while (true) {                                                \
                vTaskDelay(pdMS_TO_TICKS(1000));                          \
            }                                                             \
        }                                                                 \
    } while(0)


void app_main() {
    // Initialize I2C master
    ESP_ERROR_CHECK(i2c_master_init());

    // Scan for sensors on the I2C bus
    esp_err_t err = i2c_scan_for_sensors(sensors, &sensor_count);
    if (err == ESP_OK) {
        bool lsm6dsox_found = false;
        bool lis3mdl_found = false;

        for (int i = 0; i < sensor_count; i++) {
            if (sensors[i].type == SENSOR_LSM6DSOX) {
                ESP_LOGI(TAG, "LSM6DSOX found at address 0x%02x", sensors[i].address);
                ESP_ERROR_CHECK_CUSTOM(lsm6dsox_init(I2C_MASTER_NUM, sensors[i].address));
                lsm6dsox_found = true;
                sensor_address = sensors[i].address;
            } else if (sensors[i].type == SENSOR_LIS3MDL) {
                ESP_LOGI(TAG, "LIS3MDL found at address 0x%02x", sensors[i].address);
                ESP_ERROR_CHECK_CUSTOM(lis3mdl_init(I2C_MASTER_NUM, sensors[i].address));
                lis3mdl_found = true;
            } else {
                ESP_LOGI(TAG, "Unknown sensor found at address 0x%02x", sensors[i].address);
            }
        }

        // If both sensors are found, configure GPIO interrupts and create the sensor task
        if (lsm6dsox_found && lis3mdl_found) {
            configure_gpio_interrupts();

            // Create the sensor task pinned to Core 1
            xTaskCreatePinnedToCore(sensor_task, "sensor_task", 4096, NULL, 10, &sensor_task_handle, 1);

            vTaskDelay(100 / portTICK_PERIOD_MS);

            // Make an initial call to the ISR handler to trigger the first reads
            gpio_isr_handler((void*) LSM6SDOX_GPIO_INT_PIN);
            gpio_isr_handler((void*) LIS3MDL_GPIO_INT_PIN);
        }
    } else {
        ESP_LOGE(TAG, "No sensors found on I2C bus");
    }
}



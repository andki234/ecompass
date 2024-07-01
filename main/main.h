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
    
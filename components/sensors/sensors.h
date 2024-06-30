#ifndef SENSOR_H
#define SENSOR_H

#include "driver/i2c.h"
#include "esp_err.h"

// Define sensor types
typedef enum {
    SENSOR_UNKNOWN,
    SENSOR_LSM6DSOX,
    SENSOR_LIS3MDL
} sensor_type_t;

#endif // SENSOR_H

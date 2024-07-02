#ifndef JSON_H
#define JSON_H

#include <stdint.h>

// Function declarations
void print_json_mag_data(int64_t sampletime, int x, int y, int z);
void print_json_gyro_acc_data(int64_t sampletime, int acc_x, int acc_y, int acc_z, int gyro_x, int gyro_y, int gyro_z);

#endif // JSON_H

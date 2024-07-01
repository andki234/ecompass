#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cJSON.h>
#include "esp_log.h"
#include "json.h"

static const char *TAG = "JSON_PRINT";

void print_mag_data(int64_t timestamp, int x, int y, int z) {
    cJSON *root = cJSON_CreateObject();
    cJSON *timestamps = cJSON_CreateArray();
    cJSON *mag_data = cJSON_CreateArray();

    cJSON_AddItemToObject(root, "timestamps", timestamps);
    cJSON_AddItemToObject(root, "mag_data", mag_data);

    cJSON_AddItemToArray(timestamps, cJSON_CreateNumber(timestamp));

    cJSON *mag_data_item = cJSON_CreateObject();
    cJSON_AddNumberToObject(mag_data_item, "x", x);
    cJSON_AddNumberToObject(mag_data_item, "y", y);
    cJSON_AddNumberToObject(mag_data_item, "z", z);
    cJSON_AddItemToArray(mag_data, mag_data_item);

    char *json_data = cJSON_Print(root);
    cJSON_Delete(root);

    ESP_LOGI(TAG, "Magnetometer JSON data: %s", json_data);

    free(json_data);
}

void print_gyro_acc_data(int64_t timestamp, int acc_x, int acc_y, int acc_z, int gyro_x, int gyro_y, int gyro_z) {
    cJSON *root = cJSON_CreateObject();
    cJSON *timestamps = cJSON_CreateArray();
    cJSON *acc_data = cJSON_CreateArray();
    cJSON *gyro_data = cJSON_CreateArray();

    cJSON_AddItemToObject(root, "timestamps", timestamps);
    cJSON_AddItemToObject(root, "acc_data", acc_data);
    cJSON_AddItemToObject(root, "gyro_data", gyro_data);

    cJSON_AddItemToArray(timestamps, cJSON_CreateNumber(timestamp));

    cJSON *acc_data_item = cJSON_CreateObject();
    cJSON_AddNumberToObject(acc_data_item, "x", acc_x);
    cJSON_AddNumberToObject(acc_data_item, "y", acc_y);
    cJSON_AddNumberToObject(acc_data_item, "z", acc_z);
    cJSON_AddItemToArray(acc_data, acc_data_item);

    cJSON *gyro_data_item = cJSON_CreateObject();
    cJSON_AddNumberToObject(gyro_data_item, "x", gyro_x);
    cJSON_AddNumberToObject(gyro_data_item, "y", gyro_y);
    cJSON_AddNumberToObject(gyro_data_item, "z", gyro_z);
    cJSON_AddItemToArray(gyro_data, gyro_data_item);

    char *json_data = cJSON_Print(root);
    cJSON_Delete(root);

    ESP_LOGI(TAG, "Accelerometer and Gyroscope JSON data: %s", json_data);

    free(json_data);
}

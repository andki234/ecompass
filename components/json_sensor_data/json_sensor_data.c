/**
 * @file json_sensor_data.c
 * @brief This file contains functions for printing sensor data in JSON format.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cJSON.h>
#include "esp_log.h"

static const char *TAG = "JSON_PRINT";

/**
 * @brief Prints magnetometer data in JSON format.
 * 
 * This function takes the sample time, x, y, and z values of the magnetometer data
 * and prints them in JSON format.
 * 
 * @param sampletime The sample time of the magnetometer data.
 * @param x The x value of the magnetometer data.
 * @param y The y value of the magnetometer data.
 * @param z The z value of the magnetometer data.
 */
void print_json_mag_data(int64_t sampletime, int x, int y, int z) {
    // Create JSON objects
    cJSON *root = cJSON_CreateObject();
    cJSON *sampletimes = cJSON_CreateArray();
    cJSON *mag_data = cJSON_CreateArray();

    // Add sampletime and mag_data arrays to the root object
    cJSON_AddItemToObject(root, "sampletime", sampletimes);
    cJSON_AddItemToObject(root, "mag_data", mag_data);

    // Add sampletime to the sampletimes array
    cJSON_AddItemToArray(sampletimes, cJSON_CreateNumber(sampletime));

    // Create a JSON object for mag_data and add x, y, and z values
    cJSON *mag_data_item = cJSON_CreateObject();
    cJSON_AddNumberToObject(mag_data_item, "x", x);
    cJSON_AddNumberToObject(mag_data_item, "y", y);
    cJSON_AddNumberToObject(mag_data_item, "z", z);
    cJSON_AddItemToArray(mag_data, mag_data_item);

    // Print the JSON data
    char *json_data = cJSON_PrintUnformatted(root); // Use cJSON_PrintUnformatted to print in one line
    cJSON_Delete(root);
    printf("%s\n", json_data);
    //ESP_LOGI(TAG, "Magnetometer JSON data: %s", json_data);
    free(json_data);
}

/**
 * @brief Prints accelerometer and gyroscope data in JSON format.
 * 
 * This function takes the sample time, accelerometer (acc) x, y, and z values,
 * and gyroscope (gyro) x, y, and z values and prints them in JSON format.
 * 
 * @param sampletime The sample time of the accelerometer and gyroscope data.
 * @param acc_x The x value of the accelerometer data.
 * @param acc_y The y value of the accelerometer data.
 * @param acc_z The z value of the accelerometer data.
 * @param gyro_x The x value of the gyroscope data.
 * @param gyro_y The y value of the gyroscope data.
 * @param gyro_z The z value of the gyroscope data.
 */
void print_json_gyro_acc_data(int64_t sampletime, int acc_x, int acc_y, int acc_z, int gyro_x, int gyro_y, int gyro_z) {
    // Create JSON objects
    cJSON *root = cJSON_CreateObject();
    cJSON *sampletimes = cJSON_CreateArray();
    cJSON *acc_data = cJSON_CreateArray();
    cJSON *gyro_data = cJSON_CreateArray();

    // Add sampletime, acc_data, and gyro_data arrays to the root object
    cJSON_AddItemToObject(root, "sampletime", sampletimes);
    cJSON_AddItemToObject(root, "acc_data", acc_data);
    cJSON_AddItemToObject(root, "gyro_data", gyro_data);

    // Add sampletime to the sampletimes array
    cJSON_AddItemToArray(sampletimes, cJSON_CreateNumber(sampletime));

    // Create a JSON object for acc_data and add acc_x, acc_y, and acc_z values
    cJSON *acc_data_item = cJSON_CreateObject();
    cJSON_AddNumberToObject(acc_data_item, "x", acc_x);
    cJSON_AddNumberToObject(acc_data_item, "y", acc_y);
    cJSON_AddNumberToObject(acc_data_item, "z", acc_z);
    cJSON_AddItemToArray(acc_data, acc_data_item);

    // Create a JSON object for gyro_data and add gyro_x, gyro_y, and gyro_z values
    cJSON *gyro_data_item = cJSON_CreateObject();
    cJSON_AddNumberToObject(gyro_data_item, "x", gyro_x);
    cJSON_AddNumberToObject(gyro_data_item, "y", gyro_y);
    cJSON_AddNumberToObject(gyro_data_item, "z", gyro_z);
    cJSON_AddItemToArray(gyro_data, gyro_data_item);

    // Print the JSON data
    char *json_data = cJSON_PrintUnformatted(root); // Use cJSON_PrintUnformatted to print in one line
    cJSON_Delete(root);
    
    printf("%s\n", json_data);
    //ESP_LOGI(TAG, "Accelerometer and Gyroscope JSON data: %s", json_data);
    free(json_data);
}


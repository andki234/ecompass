#ifndef ECOMPASS_H
#define ECOMPASS_H

#include <math.h>
#include <stdio.h>

// Kalman filter variables
typedef struct {
    float q; // Process noise covariance
    float r; // Measurement noise covariance
    float x; // Value
    float p; // Estimation error covariance
    float k; // Kalman gain
} KalmanFilter;

void kalman_init(KalmanFilter *kf, float q, float r, float initial_value);
float kalman_update(KalmanFilter *kf, float measurement);
void calculate_euler_angles(float acc_x, float acc_y, float acc_z, float* roll, float* pitch);
void insert_accel_gyro_data(float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float sample_rate);
void insert_mag_data(float mag_x, float mag_y, float mag_z, float sample_rate);
float get_heading();
void reset_sensor_calibration();
void calibrate_gyro_stationary(float gyro_x, float gyro_y, float gyro_z);

#endif // ECOMPASS_H

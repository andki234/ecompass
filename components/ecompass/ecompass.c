#include "ecompass.h"
#include "esp_log.h"

// Set TAG for logging
static const char *TAG = "ECOMPASS";

#define GYRO_SENSITIVITY 228.57   // LSB/dps for ±125 dps (LIS6DSOX)
#define ACC_SENSITIVITY 16384.0   // LSB/g for ±2 g (LIS6DSOX)
#define MAG_SENSITIVITY 6842.0    // LSB/gauss for ±4 gauss (LIS3MDL)

// Noise parameters from datasheet
#define GYROSCOPE_NOISE (4e-3 * 4e-3) // (rad/s)^2
#define ACCELEROMETER_NOISE (90e-6 * 90e-6) // (m/s^2)^2
#define MAGNETOMETER_NOISE (3.5e-3 * 3.5e-3) // (gauss)^2

#define ALPHA 0.1 // Smoothing factor for exponential moving average
#define BETA 0.98 // Complementary filter coefficient for gyroscope yaw blending

static float pitch = 0.0, roll = 0.0, yaw = 0.0, gyro_yaw = 0.0;
static KalmanFilter kf_pitch, kf_roll, kf_yaw;
static int initialized = 0;

typedef struct {
    float bias_x;
    float bias_y;
    float bias_z;
    float scale_x;
    float scale_y;
    float scale_z;
} CalibrationData;

static CalibrationData accel_gyro_cal = {0};
static CalibrationData mag_cal = {0};

typedef struct {
    float sum_x;
    float sum_y;
    float sum_z;
    int count;
} GyroCalibrationData;

static GyroCalibrationData gyro_cal_data = {0};

// For smoothing
static float mag_x_smooth = 0.0, mag_y_smooth = 0.0, mag_z_smooth = 0.0;

void kalman_init(KalmanFilter *kf, float q, float r, float initial_value) {
    kf->q = q;
    kf->r = r;
    kf->x = initial_value;
    kf->p = 1.0;
    kf->k = 0.0;
}

float kalman_update(KalmanFilter *kf, float measurement) {
    // Prediction update
    kf->p = kf->p + kf->q;

    // Measurement update
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1 - kf->k) * kf->p;

    return kf->x;
}

void calculate_euler_angles(float acc_x, float acc_y, float acc_z, float* roll, float* pitch) {
    // Calculate roll (phi) and pitch (theta) from accelerometer data
    *roll = atan2(acc_y, acc_z);
    *pitch = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z));
}

void apply_calibration(float* x, float* y, float* z, CalibrationData* cal) {
    *x = (*x - cal->bias_x);
    *y = (*y - cal->bias_y);
    *z = (*z - cal->bias_z);
}

void insert_accel_gyro_data(float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float sample_rate) {
    if (!initialized) {
        kalman_init(&kf_pitch, ACCELEROMETER_NOISE, GYROSCOPE_NOISE, 0.0);
        kalman_init(&kf_roll, ACCELEROMETER_NOISE, GYROSCOPE_NOISE, 0.0);
        kalman_init(&kf_yaw, MAGNETOMETER_NOISE, GYROSCOPE_NOISE, 0.0);
        initialized = 1;
    }

    float dt = 1.0 / sample_rate;

    // Apply calibration to accelerometer data
    apply_calibration(&acc_x, &acc_y, &acc_z, &accel_gyro_cal);

    // Sensitivity scaling to accelerometer data
    acc_x /= ACC_SENSITIVITY;
    acc_y /= ACC_SENSITIVITY;
    acc_z /= ACC_SENSITIVITY;

    // Normalize accelerometer data
    float acc_norm = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
    acc_x /= acc_norm;
    acc_y /= acc_norm;
    acc_z /= acc_norm;

    // Calculate pitch and roll from accelerometer data
    float pitch_acc, roll_acc;
    calculate_euler_angles(acc_x, acc_y, acc_z, &roll_acc, &pitch_acc);

    // Update Kalman filters for pitch and roll
    pitch = kalman_update(&kf_pitch, pitch_acc);
    roll = kalman_update(&kf_roll, roll_acc);

    // Apply calibration to gyroscope data
    apply_calibration(&gyro_x, &gyro_y, &gyro_z, &accel_gyro_cal);

    // Sensitivity scaling to gyroscope data
    gyro_x /= GYRO_SENSITIVITY;
    gyro_y /= GYRO_SENSITIVITY;
    gyro_z /= GYRO_SENSITIVITY;

    // Integrate gyroscope data to update yaw
    gyro_yaw -= gyro_z * dt; // Adjust sign for correct direction
    gyro_yaw = fmod(gyro_yaw, 360.0);
    if (gyro_yaw < 0) gyro_yaw += 360.0;

    // Update the complementary filter for yaw using gyro data
    yaw = BETA * (yaw + gyro_z * dt) + (1 - BETA) * yaw;

    // Print Euler angles for verification
    printf("\033[32mPitch: %.2f degrees\tRoll: %.2f degrees\tGyro Yaw: %.2f degrees\tMagnetic Yaw: %.2f\033[0m\r", pitch * 180.0 / M_PI, roll * 180.0 / M_PI, gyro_yaw, yaw);

}

void insert_mag_data(float mag_x, float mag_y, float mag_z, float sample_rate) {
    if (!initialized) {
        kalman_init(&kf_pitch, ACCELEROMETER_NOISE, GYROSCOPE_NOISE, 0.0);
        kalman_init(&kf_roll, ACCELEROMETER_NOISE, GYROSCOPE_NOISE, 0.0);
        kalman_init(&kf_yaw, MAGNETOMETER_NOISE, GYROSCOPE_NOISE, 0.0);
        initialized = 1;
    }

    // Apply calibration to magnetometer data
    apply_calibration(&mag_x, &mag_y, &mag_z, &mag_cal);

    // Sensitivity scaling to magnetometer data
    mag_x /= MAG_SENSITIVITY;
    mag_y /= MAG_SENSITIVITY;
    mag_z /= MAG_SENSITIVITY;

    // Exponential moving average for smoothing
    mag_x_smooth = ALPHA * mag_x + (1 - ALPHA) * mag_x_smooth;
    mag_y_smooth = ALPHA * mag_y + (1 - ALPHA) * mag_y_smooth;
    mag_z_smooth = ALPHA * mag_z + (1 - ALPHA) * mag_z_smooth;

    // Normalize magnetometer data
    float mag_norm = sqrt(mag_x_smooth * mag_x_smooth + mag_y_smooth * mag_y_smooth + mag_z_smooth * mag_z_smooth);
    mag_x_smooth /= mag_norm;
    mag_y_smooth /= mag_norm;
    mag_z_smooth /= mag_norm;

    // Tilt compensation for magnetometer
    float mag_x_comp = mag_x_smooth * cos(pitch) + mag_y_smooth * sin(roll) * sin(pitch) + mag_z_smooth * cos(roll) * sin(pitch);
    float mag_y_comp = mag_y_smooth * cos(roll) - mag_z_smooth * sin(roll);

    // Calculate heading
    float heading = atan2(mag_y_comp, mag_x_comp);

    // Convert heading from radians to degrees
    heading *= 180.0 / M_PI;

    // Normalize heading to 0 - 360 degrees
    if (heading < 0) {
        heading += 360.0;
    }

    // Apply complementary filter correction for yaw using magnetometer data
    yaw = BETA * yaw + (1 - BETA) * heading;
}

float get_heading() {
    return yaw;
}

float get_gyro_heading() {
    return gyro_yaw;
}

void reset_sensor_calibration() {
    // Initialize calibration data with zeros and scale factors as 1 (no scaling)
    accel_gyro_cal.bias_x = 0.0;
    accel_gyro_cal.bias_y = 0.0;
    accel_gyro_cal.bias_z = 0.0;
    accel_gyro_cal.scale_x = 1.0;
    accel_gyro_cal.scale_y = 1.0;
    accel_gyro_cal.scale_z = 1.0;

    gyro_cal_data.sum_x = 0.0;
    gyro_cal_data.sum_y = 0.0;
    gyro_cal_data.sum_z = 0.0;
    gyro_cal_data.count = 0;
    
    mag_cal.bias_x = 0.0;
    mag_cal.bias_y = 0.0;
    mag_cal.bias_z = 0.0;
    mag_cal.scale_x = 1.0;
    mag_cal.scale_y = 1.0;
    mag_cal.scale_z = 1.0;
}

void calibrate_gyro_stationary(float gyro_x, float gyro_y, float gyro_z) {
    // Update sum and count with new data
    gyro_cal_data.sum_x += gyro_x;
    gyro_cal_data.sum_y += gyro_y;
    gyro_cal_data.sum_z += gyro_z;
    gyro_cal_data.count++;

    // Calculate the average (bias) values when enough samples are collected
    if (gyro_cal_data.count > 0) {
        accel_gyro_cal.bias_x = gyro_cal_data.sum_x / gyro_cal_data.count;
        accel_gyro_cal.bias_y = gyro_cal_data.sum_y / gyro_cal_data.count;
        accel_gyro_cal.bias_z = gyro_cal_data.sum_z / gyro_cal_data.count;

        printf("Gyro calibration in progress: Bias_x=%.6f, Bias_y=%.6f, Bias_z=%.6f\n", accel_gyro_cal.bias_x, accel_gyro_cal.bias_y, accel_gyro_cal.bias_z);
    }
}

#ifndef MAG_CALIBRATION_H
#define MAG_CALIBRATION_H

void mag_calibration(float raw_x, float raw_y, float raw_z, float *cal_x, float *cal_y, float *cal_z);
float calculate_heading(float mag_x, float mag_y, float mag_z, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float dt);

#endif // MAG_CALIBRATION_H

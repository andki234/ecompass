#include "mag_calibration.h"
#include <math.h>

// Calibration matrix and offset vector
static const float A[3][3] = {
    {1.133004436054362, 0.035382153992458, -0.059458723886203},
    {0.035382153992458, 0.942370110979664, -0.057914195518959},
    {-0.059458723886203, -0.057914195518959, 0.944142609400317}
};

static const float b[3] = {-0.096459982696593, -0.066528083920667, -0.325974886182747};

void mag_calibration(float raw_x, float raw_y, float raw_z, float *cal_x, float *cal_y, float *cal_z) {
    float raw[3] = {raw_x, raw_y, raw_z};
    float cal[3] = {0};

    for (int i = 0; i < 3; i++) {
        cal[i] = b[i];
        for (int j = 0; j < 3; j++) {
            cal[i] += A[i][j] * raw[j];
        }
    }

    *cal_x = cal[0];
    *cal_y = cal[1];
    *cal_z = cal[2];
}
#include "mag_calibration.h"
#include <math.h>

// Calibration matrix and offset vector
static const float A[3][3] = {
    {1.133004436054362, 0.035382153992458, -0.059458723886203},
    {0.035382153992458, 0.942370110979664, -0.057914195518959},
    {-0.059458723886203, -0.057914195518959, 0.944142609400317}
};

static const float b[3] = {-0.096459982696593, -0.066528083920667, -0.325974886182747};

// Inline keyword suggests the compiler to inline this function, reducing call overhead
inline void mag_calibration(float raw_x, float raw_y, float raw_z, float *cal_x, float *cal_y, float *cal_z) {
    float raw[3] = {raw_x, raw_y, raw_z};

    // Ensure the calculations leverage FPU by avoiding unnecessary operations
    *cal_x = b[0] + A[0][0] * raw[0] + A[0][1] * raw[1] + A[0][2] * raw[2];
    *cal_y = b[1] + A[1][0] * raw[0] + A[1][1] * raw[1] + A[1][2] * raw[2];
    *cal_z = b[2] + A[2][0] * raw[0] + A[2][1] * raw[1] + A[2][2] * raw[2];
}

#include "coordinate_transform.h"
#include <math.h>

void body_rotation_matrix(
    double yaw_rad,
    double pitch_rad,
    double roll_rad,
    double R_WB[3][3]
) {

    double cos_roll = cos(roll_rad);
    double sin_roll = sin(roll_rad);
    double cos_pitch = cos(pitch_rad);
    double sin_pitch = sin(pitch_rad);
    double cos_yaw = cos(yaw_rad);
    double sin_yaw = sin(yaw_rad);

    //Matrices use [row][column]; indices 0, 1, 2
    double R_x[3][3] = {
        {1, 0,        0},
        {0, cos_roll, -sin_roll},
        {0, sin_roll,  cos_roll}
    };

    double R_y[3][3] = {
        { cos_pitch, 0, sin_pitch},
        { 0,         1, 0},
        {-sin_pitch, 0, cos_pitch}
    };

    double R_z[3][3] = {
        {cos_yaw, -sin_yaw, 0},
        {sin_yaw,  cos_yaw, 0},
        {0,        0,       1}
    };

    //First product: R_zy = R_z * R_y (one row times one column)
    double R_zy[3][3];
    for (int row = 0; row < 3; row++) {
        for (int column = 0; column < 3; column++) {
            R_zy[row][column] = R_z[row][0] * R_y[0][column]
                              + R_z[row][1] * R_y[1][column]
                              + R_z[row][2] * R_y[2][column];
        }
    }

    //Second product: R_WB = R_zy * R_x
    for (int row = 0; row < 3; row++) {
        for (int column = 0; column < 3; column++) {
            R_WB[row][column] = R_zy[row][0] * R_x[0][column]
                              + R_zy[row][1] * R_x[1][column]
                              + R_zy[row][2] * R_x[2][column];
        }
    }
}

HipFootPosition foot_world_to_hip(
    WorldFootPosition foot_bottom_W,
    double ball_radius_m,
    const double body_position_W_m[3],
    const double R_WB[3][3],
    const double hip_position_B_m[3]
) {

    //1. c_W = p_W + {0, 0, ball_radius}; lift along world z
    double centre_W[3] = {
        foot_bottom_W.x_W_m,
        foot_bottom_W.y_W_m,
        foot_bottom_W.z_W_m + ball_radius_m
    };

    //2. Subtract the body origin, still expressed in W
    double relative_W[3] = {
        centre_W[0] - body_position_W_m[0],
        centre_W[1] - body_position_W_m[1],
        centre_W[2] - body_position_W_m[2]
    };

    //c_B = transpose(R_WB) * relative_W; use columns of R_WB
    double centre_B[3];
    centre_B[0] = R_WB[0][0] * relative_W[0] + R_WB[1][0] * relative_W[1] + R_WB[2][0] * relative_W[2];
    centre_B[1] = R_WB[0][1] * relative_W[0] + R_WB[1][1] * relative_W[1] + R_WB[2][1] * relative_W[2];
    centre_B[2] = R_WB[0][2] * relative_W[0] + R_WB[1][2] * relative_W[1] + R_WB[2][2] * relative_W[2];

    //3. c_H = c_B - hip_B; our fixed H axes are parallel to B
    HipFootPosition foot_centre_H;
    foot_centre_H.x_H_m = centre_B[0] - hip_position_B_m[0];
    foot_centre_H.y_H_m = centre_B[1] - hip_position_B_m[1];
    foot_centre_H.z_H_m = centre_B[2] - hip_position_B_m[2];

    return foot_centre_H;
}

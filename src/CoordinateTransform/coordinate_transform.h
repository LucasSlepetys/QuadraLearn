#ifndef COORDINATE_TRANSFORM
#define COORDINATE_TRANSFORM

#include "../FootTrajectory/foot_trajectory.h"

typedef struct {
    double ball_radius_m;
    double hip_position_B_m[4][3]; //FL, FR, RL, RR; each row is {x, y, z} in B
    double body_height_m; //Body-origin height above ground
    double body_roll_rad;
    double body_pitch_rad;
} CoordinateTransformConfig;

typedef struct {
    double x_H_m;
    double y_H_m;
    double z_H_m;
} HipFootPosition; //Ball centre in the fixed hip-roll frame H [m]

//Fill R_WB = R_z(yaw) * R_y(pitch) * R_x(roll)
void body_rotation_matrix(
    double yaw_rad,
    double pitch_rad,
    double roll_rad,
    double R_WB[3][3]
);

//Convert the world ball-bottom target into a hip-frame ball-centre target
HipFootPosition foot_world_to_hip(
    WorldFootPosition foot_bottom_W,
    double ball_radius_m,
    const double body_position_W_m[3],
    const double R_WB[3][3],
    const double hip_position_B_m[3]
);

#endif // COORDINATE_TRANSFORM

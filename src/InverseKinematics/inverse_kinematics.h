#ifndef INVERSE_KINEMATICS
#define INVERSE_KINEMATICS

#include "../CoordinateTransform/coordinate_transform.h"

typedef struct {
    double upper_link_length_m; //L1: projected upper-link length
    double lower_link_length_m; //L2: projected lower-link length
    double hip_roll_to_pitch_m[4][3]; //FL, FR, RL, RR; {d_x, d_y, d_z} in M
    double upper_lateral_offset_m[4]; //a1, signed along unrolled +y
    double lower_lateral_offset_m[4]; //a2, signed along unrolled +y
} InverseKinematicsConfig;

typedef struct {
    double alpha_rad;   //Hip roll, right-hand rotation about +x_H
    double theta_1_rad; //Hip pitch, from downward towards +x
    double theta_2_rad; //Knee relative to upper link; positive-knee branch
} LegJointAngles;

//Assumes reachable targets: r^2 > 0 and |L1-L2| < D < L1+L2
LegJointAngles leg_inverse_kinematics(
    HipFootPosition foot_center_H,
    double upper_link_length_m,
    double lower_link_length_m,
    const double hip_roll_to_pitch_m[3],
    double upper_lateral_offset_m,
    double lower_lateral_offset_m
);

#endif // INVERSE_KINEMATICS

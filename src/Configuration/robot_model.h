#ifndef ROBOT_MODEL
#define ROBOT_MODEL

#include "../InverseKinematics/inverse_kinematics.h"

//Measured geometry only; all lengths are in metres
//Use FL, FR, RL, RR order throughout the project
extern const double robot_ball_radius_m;
extern const double robot_hip_position_B_m[4][3];
extern const InverseKinematicsConfig ik_config;

#endif // ROBOT_MODEL

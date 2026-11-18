#include "chosen_configurations.h"
#include "robot_model.h"

//Test commands for now; later these values can arrive from an upstream module
const MotionCommand initial_command = {
    0.1,  //Forward velocity [m/s]
    0.44,   //Yaw rate [rad/s]
    0.145, //Body-origin height above ground [m]
    0.0,   //Body roll [rad]
    0.0    //Body pitch [rad]
};

//Chosen gait settings
const double gait_period_s = 0.5;
const double stance_fraction = 0.5;
const GaitConfig gait = {
    gait_period_s,
    stance_fraction,
    {0.0, gait_period_s / 2, gait_period_s / 2, 0.0} //FL, FR, RL, RR trot offsets
};

const double swing_height_m = 0.02; //Maximum ball-bottom clearance [m]

//Nominal foot placement relative to each hip reference in the level G layout
const double x_0_m = 0.0;  //Positive forwards, negative backwards [m]
const double y_0_m = 0.0; //Positive outwards, negative inwards [m]; not a CAD link offset
const double ground_z_W_m = 0.0;

//Desktop simulation settings; these do not command a travelled distance
const double dt_s = 0.02;
const int simulation_updates = 500; //50 = 1 sec fpr dt_s = 0.02
const BodyPose2D initial_body_pose = {0.0, 0.0, 0.0}; //World x, y and yaw; separate from x_0/y_0

FootPlacementConfig make_foot_placement_config(
    double x_0_m,
    double y_0_m,
    double ground_z_W_m
) {

    FootPlacementConfig foot_config;
    const int side[4] = {1, -1, 1, -1}; //Left +1, right -1

    //Nominal G layout: f_ix = h_ix + x_0; f_iy = h_iy + s_i*y_0
    for (int i = 0; i < 4; i++) {
        foot_config.nominal_foot_x_G_m[i] = robot_hip_position_B_m[i][0] + x_0_m;
        foot_config.nominal_foot_y_G_m[i] = robot_hip_position_B_m[i][1] + side[i] * y_0_m;
    }
    foot_config.ground_z_W_m = ground_z_W_m;

    return foot_config;
}

CoordinateTransformConfig make_coordinate_transform_config(
    const MotionCommand *command
) {

    CoordinateTransformConfig transform_config;
    transform_config.ball_radius_m = robot_ball_radius_m;

    //Copy the measured hip locations, each row is {x, y, z} in B
    for (int i = 0; i < 4; i++) {
        for (int coordinate = 0; coordinate < 3; coordinate++) {
            transform_config.hip_position_B_m[i][coordinate] = robot_hip_position_B_m[i][coordinate];
        }
    }

    //The desired body posture comes from the current command
    transform_config.body_height_m = command->body_height_m;
    transform_config.body_roll_rad = command->body_roll_rad;
    transform_config.body_pitch_rad = command->body_pitch_rad;

    return transform_config;
}

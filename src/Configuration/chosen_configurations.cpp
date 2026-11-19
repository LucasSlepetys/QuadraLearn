#include "chosen_configurations.h"
#include "robot_model.h"

//Test commands for now; later these values can arrive from an upstream module
const MotionCommand initial_command = {
    0.01,  //Forward velocity [m/s]
    0.4,   //Yaw rate [rad/s]
    0.145, //Target body height after the grounded adjustment [m]
    0.00295740232396, //Body roll [rad]: +0.16945 deg makes all CAD-home feet coplanar
    0.0    //Body pitch [rad]
};

//Chosen gait settings
const double gait_period_s = 1;
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
const int simulation_updates = 1000; //20 seconds: stand, adjust height, start, walk, stop, hold
const double standing_delay_s = 1; //Hold the initial pose before adjusting height [real s]
const double height_adjustment_duration_s = 2.0; //T_H: lower/raise with four feet grounded [real s]
const double start_ramp_duration_s = 3; //T_a: gradual acceleration duration [real s]
const double stop_request_time_s = 17.0; //Desktop STOP at this real time [s]
const double minimum_stop_duration_s = 1.0; //T_min; actual ramp ends at a landing [real s]
const BodyPose2D initial_body_pose = {0.0, 0.0, 0.0}; //World x, y and yaw; separate from x_0/y_0

//Desktop starting inputs: all four legs at CAD home; later supply measured model angles.
//Edit degrees below; stored values are radians, not physical servo angles.
//At the small body roll above, FK recovers H0 = 0.081196102 m with four feet grounded.
const double degrees_to_radians = 3.141592653589793 / 180.0;
const LegJointAngles initial_joint_angles[4] = {
    // alpha (hip roll),                 theta_1 (hip pitch),              theta_2 (relative knee)
    {-30.0 * degrees_to_radians, -104.5669876439 * degrees_to_radians, 134.1685941347 * degrees_to_radians}, //FL
    { 30.0 * degrees_to_radians, -104.5669876439 * degrees_to_radians, 134.1685943331 * degrees_to_radians}, //FR
    {-30.0 * degrees_to_radians, -104.5669876439 * degrees_to_radians, 134.1685941347 * degrees_to_radians}, //RL
    { 30.0 * degrees_to_radians, -104.5669876439 * degrees_to_radians, 134.1685943331 * degrees_to_radians}  //RR
};

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

    //Initial settings; initialization and the height ramp then supply the current H
    transform_config.body_height_m = command->body_height_m;
    transform_config.body_roll_rad = command->body_roll_rad;
    transform_config.body_pitch_rad = command->body_pitch_rad;

    return transform_config;
}

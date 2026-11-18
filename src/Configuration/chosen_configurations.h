#ifndef CHOSEN_CONFIGURATIONS
#define CHOSEN_CONFIGURATIONS

#include "../BodyMotion/body_motion.h"
#include "../Gait/gait.h"
#include "../FootPlacement/foot_placement.h"
#include "../CoordinateTransform/coordinate_transform.h"
#include "../InverseKinematics/inverse_kinematics.h"

//A future upstream controller can supply these inputs before each update
typedef struct {
    double forward_velocity_m_s; //Along +x_G
    double yaw_rate_rad_s;       //About world vertical +z_W
    double body_height_m;        //Requested body-origin height above ground [m]
    double body_roll_rad;        //phi in R_WB = Rz(yaw) * Ry(pitch) * Rx(roll)
    double body_pitch_rad;       //beta in the same rotation
} MotionCommand;

extern const MotionCommand initial_command; //Desktop test inputs for now
extern const GaitConfig gait;
extern const double swing_height_m;
extern const double x_0_m; //Common fore/aft placement offset in G
extern const double y_0_m; //Common signed outward placement offset in G
extern const double ground_z_W_m;
extern const double dt_s;
extern const int simulation_updates;
extern const double standing_delay_s; //Real hold before height preparation
extern const double height_adjustment_duration_s; //T_H: real seconds to reach the requested height
extern const double start_ramp_duration_s; //T_a: real seconds to reach normal rate
extern const double stop_request_time_s; //Real time of the desktop STOP request
extern const double minimum_stop_duration_s; //T_min: may be extended to reach a landing
extern const BodyPose2D initial_body_pose;
extern const LegJointAngles initial_joint_angles[4]; //FL, FR, RL, RR; model angles [rad]

FootPlacementConfig make_foot_placement_config(
    double x_0_m,
    double y_0_m,
    double ground_z_W_m
);

CoordinateTransformConfig make_coordinate_transform_config(
    const MotionCommand *command
);

#endif // CHOSEN_CONFIGURATIONS

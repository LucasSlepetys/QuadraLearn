#ifndef CHOSEN_CONFIGURATIONS
#define CHOSEN_CONFIGURATIONS

#include "../BodyMotion/body_motion.h"
#include "../Gait/gait.h"
#include "../FootPlacement/foot_placement.h"
#include "../CoordinateTransform/coordinate_transform.h"

//A future upstream controller can supply these inputs before each update
typedef struct {
    double forward_velocity_m_s; //Along +x_G
    double yaw_rate_rad_s;       //About world vertical +z_W
    double body_height_m;        //Body-origin height above ground
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
extern const BodyPose2D initial_body_pose;

FootPlacementConfig make_foot_placement_config(
    double x_0_m,
    double y_0_m,
    double ground_z_W_m
);

CoordinateTransformConfig make_coordinate_transform_config(
    const MotionCommand *command
);

#endif // CHOSEN_CONFIGURATIONS

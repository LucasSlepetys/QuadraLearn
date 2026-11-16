#ifndef LOCOMOTION
#define LOCOMOTION

#include <stdio.h>
#include "../BodyMotion/body_motion.h"
#include "../Gait/gait.h"
#include "../FootPlacement/foot_placement.h"
#include "../FootTrajectory/foot_trajectory.h"
#include "../CoordinateTransform/coordinate_transform.h"
#include "../InverseKinematics/inverse_kinematics.h"

/* Keeping Q in memory:
WorldFootprint contact_W;
WorldFootprint swing_start_W;
WorldFootprint swing_end_W; */
typedef struct {
    WorldFootprint contact_W; // The fixed world footprint used during stance
    WorldFootprint swing_start_W; //The ground point from which the current swing starts
    WorldFootprint swing_end_W; //The destination returned by foot_placement_at_liftoff
} LegFootMemory;

/*
    double time_s;
    BodyPose2D body_pose;
    GaitPhaseResult leg_phase[4];
    LegFootMemory foot[4]; 
    WorldFootPosition foot_position_W[4];
    HipFootPosition foot_center_H[4];
    LegJointAngles joint_angles[4];
*/
typedef struct {
    double time_s;
    BodyPose2D body_pose;
    GaitPhaseResult leg_phase[4]; //Gait phase result for each leg
    LegFootMemory foot[4];
    WorldFootPosition foot_position_W[4]; //Latest ball-bottom positions in W
    HipFootPosition foot_center_H[4]; //Ball-centre targets for IK, in each leg's H frame
    LegJointAngles joint_angles[4]; //Latest alpha, theta_1 and theta_2 [rad]
} LocomotionState; //Elapsed time and planned body pose for that time

//Set the initial phases and footprints for the mathematical gait at time zero
void locomotion_initialize(
    LocomotionState *state,
    BodyPose2D initial_body_pose,
    double forward_velocity_m_s,
    double yaw_rate_rad_s,
    const GaitConfig *gait,
    const FootPlacementConfig *foot_config,
    double swing_height_m,
    const CoordinateTransformConfig *transform_config,
    const InverseKinematicsConfig *ik_config
);

/*
LocomotionState *state,
double forward_velocity_m_s,
double yaw_rate_rad_s,
double dt_s, //Shorter than both stance and swing durations
const GaitConfig *gait,
const FootPlacementConfig *foot_config,
double swing_height_m,
const CoordinateTransformConfig *transform_config,
const InverseKinematicsConfig *ik_config
*/
void locomotion_update(
    LocomotionState *state,
    double forward_velocity_m_s,
    double yaw_rate_rad_s,
    double dt_s,
    const GaitConfig *gait,
    const FootPlacementConfig *foot_config,
    double swing_height_m,
    const CoordinateTransformConfig *transform_config,
    const InverseKinematicsConfig *ik_config
);



#endif // LOCOMOTION
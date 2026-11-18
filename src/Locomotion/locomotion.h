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
    double time_s; //Real elapsed time t [s]
    double motion_time_s; //Motion time s: advances the body and gait together
    BodyPose2D body_pose;
    GaitPhaseResult leg_phase[4];
    LegFootMemory foot[4]; 
    WorldFootPosition foot_position_W[4];
    HipFootPosition foot_center_H[4];
    LegJointAngles joint_angles[4];
*/
typedef struct {
    double time_s; //Real elapsed time t [s]
    double motion_time_s; //Motion time s: advances the body and gait together
    BodyPose2D body_pose;
    GaitPhaseResult leg_phase[4]; //Gait phase result for each leg
    LegFootMemory foot[4];
    WorldFootPosition foot_position_W[4]; //Latest ball-bottom positions in W
    HipFootPosition foot_center_H[4]; //Ball-centre targets for IK, in each leg's H frame
    LegJointAngles joint_angles[4]; //Latest alpha, theta_1 and theta_2 [rad]
} LocomotionState; //Elapsed time and planned body pose for that time

/* Initialize a standing pose from q_i,0 [rad], using FK rather than IK.
 * Assumes the supplied body attitude and four feet on one flat ground plane.
 * Writes the recovered H0 into transform_config->body_height_m.
 * Returns 1 on success, 0 if the feet imply incompatible/invalid heights.
 */
int locomotion_initialize(
    LocomotionState *state,
    BodyPose2D initial_body_pose,
    const LegJointAngles initial_joint_angles[4],
    double ground_z_W_m,
    CoordinateTransformConfig *transform_config,
    const InverseKinematicsConfig *ik_config
);

//Update grounded joint angles for transform_config's current height; advance real time only.
//Requires four stance legs. Returns 0 without changing state if IK is invalid, otherwise 1.
int locomotion_update_standing(
    LocomotionState *state,
    double dt_s,
    double ground_z_W_m,
    const CoordinateTransformConfig *transform_config,
    const InverseKinematicsConfig *ik_config
);

//Call once after standing, at s = 0, before advancing the body or feet.
//Prepare the first swing from the saved contacts without changing the pose.
void locomotion_begin_walk(
    LocomotionState *state,
    double forward_velocity_m_s,
    double yaw_rate_rad_s,
    const GaitConfig *gait,
    const FootPlacementConfig *foot_config
);

/*
LocomotionState *state,
double forward_velocity_m_s,
double yaw_rate_rad_s,
double dt_s, //Real update interval
double motion_dt_s, //Motion increment; shorter than both stance and swing durations
int finish_walk, //1 only on the final landing; keep all four feet grounded
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
    double motion_dt_s,
    int finish_walk,
    const GaitConfig *gait,
    const FootPlacementConfig *foot_config,
    double swing_height_m,
    const CoordinateTransformConfig *transform_config,
    const InverseKinematicsConfig *ik_config
);



#endif // LOCOMOTION
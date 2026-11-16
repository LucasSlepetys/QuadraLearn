/*

FOR EACH UPDATE:

    Advance the simulation time by dt

    Calculate the new planned body position and orientation 
        Produces:
        - Body-origin position in the world: x_B, y_B, z_B 
        - Body orientation: yaw, pitch and roll
        - Corresponding rotation matrix: R_WB

    FOR EACH LEG:

        Determine where this leg is in its gait cycle.
            Produces:
                - STANCE or SWING
                - Progress through that phase (u)
                - Whether lift-off or touchdown just occurred

        Choose or retain this leg's ground-contact points.

            At lift-off:
            - Keep the current contact as the swing start.
            - Calculate and remember the next touchdown point.

            At touchdown:
            - Make the swing destination (that just happened) the new ground contact.

            Between these events:
            - Keep those remembered points unchanged.

        Calculate the foot position at this instant.
            Produces:
            - Ball-bottom position in the WORLD frame: p_W

        Convert that position into the coordinates needed by IK.
            Produces:
            - Ball-centre position in the HIP frame: x_H, y_H, z_H

        Calculate the three joint angles.
            Produces:
            - Hip roll: alpha
            - Hip pitch: theta_1
            - Knee: theta_2

    Keep the updated time, body pose and leg-phase records
    for the next update.

    Make the four sets of joint angles available for plotting.

*/

#include "locomotion.h"

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
) {

    state->time_s = 0;
    state->body_pose = initial_body_pose;

    double stance_duration = gait->stance_fraction * gait->period_s;

    //Current body position b_W and rotation R_WB, shared by all four legs
    double body_position_W_m[3] = {
        state->body_pose.x_W_m,
        state->body_pose.y_W_m,
        foot_config->ground_z_W_m + transform_config->body_height_m
    };

    double R_WB[3][3];
    body_rotation_matrix(
        state->body_pose.yaw_rad,
        transform_config->body_pitch_rad,
        transform_config->body_roll_rad,
        R_WB
    );

    //For each leg:
    for (int i = 0; i < 4; i++) {

        //Initial cycle time, phase and progress
        state->leg_phase[i] = gait_phase_at_time(state->time_s, gait->period_s, gait->stance_fraction, gait->phase_offset_s[i]);

        //Offsets from time zero; a negative value evaluates the preceding stance
        double contact_midpoint_offset_s = stance_duration / 2 - state->leg_phase[i].cycle_time_s;
        double next_midpoint_offset_s = contact_midpoint_offset_s + gait->period_s;

        //Evaluate both midpoint poses from the same initial pose
        BodyPose2D contact_midpoint_pose = body_pose_update(
            initial_body_pose,
            forward_velocity_m_s,
            yaw_rate_rad_s,
            contact_midpoint_offset_s
        );

        BodyPose2D next_midpoint_pose = body_pose_update(
            initial_body_pose,
            forward_velocity_m_s,
            yaw_rate_rad_s,
            next_midpoint_offset_s
        );

        //Calculate the two world footprints
        WorldFootprint contact_footprint = footprint_from_midstance_pose(
            contact_midpoint_pose,
            foot_config->nominal_foot_x_G_m[i],
            foot_config->nominal_foot_y_G_m[i],
            foot_config->ground_z_W_m
        );

        WorldFootprint next_footprint = footprint_from_midstance_pose(
            next_midpoint_pose,
            foot_config->nominal_foot_x_G_m[i],
            foot_config->nominal_foot_y_G_m[i],
            foot_config->ground_z_W_m
        );

        //Keep the initial contact and swing endpoints in memory
        state->foot[i].contact_W = contact_footprint;
        state->foot[i].swing_start_W = contact_footprint;
        state->foot[i].swing_end_W = next_footprint;

        //Calculate the current ball-bottom position in W
        state->foot_position_W[i] = foot_position_at_phase(
            state->leg_phase[i].phase,
            state->leg_phase[i].progress,
            state->foot[i].contact_W,
            state->foot[i].swing_start_W,
            state->foot[i].swing_end_W,
            swing_height_m
        );

        //Convert the ball bottom in W into the ball centre in H
        state->foot_center_H[i] = foot_world_to_hip(
            state->foot_position_W[i],
            transform_config->ball_radius_m,
            body_position_W_m,
            R_WB,
            transform_config->hip_position_B_m[i]
        );

        //Solve the three joint angles from this leg's hip-frame target
        state->joint_angles[i] = leg_inverse_kinematics(
            state->foot_center_H[i],
            ik_config->upper_link_length_m,
            ik_config->lower_link_length_m,
            ik_config->hip_roll_to_pitch_m[i],
            ik_config->upper_lateral_offset_m[i],
            ik_config->lower_lateral_offset_m[i]
        );
    }
}

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
) {

    // Update the body pose:
    state->body_pose = body_pose_update(state->body_pose, forward_velocity_m_s, yaw_rate_rad_s, dt_s); //gets new body pose
    state->time_s += dt_s; //advances time
    
    //Current body position b_W and rotation R_WB, shared by all four legs
    double body_position_W_m[3] = {
        state->body_pose.x_W_m,
        state->body_pose.y_W_m,
        foot_config->ground_z_W_m + transform_config->body_height_m
    };

    double R_WB[3][3];
    body_rotation_matrix(
        state->body_pose.yaw_rad,
        transform_config->body_pitch_rad,
        transform_config->body_roll_rad,
        R_WB
    );

    //For each leg:
    for (int i = 0; i < 4; i++) {

        // Previous phrase before update:
        Phase previous_phase = state->leg_phase[i].phase;

        // Update cycle_index, cycle_time_s, phase, progress for each leg
        state->leg_phase[i] = gait_phase_at_time(state->time_s, gait->period_s, gait->stance_fraction, gait->phase_offset_s[i]);
        
        // Phase after the update:
        Phase current_phase = state->leg_phase[i].phase;

        if(previous_phase == GAIT_STANCE && current_phase == GAIT_SWING) //Lift off occured
        {
            printf("Leg %d - Lift off at %.3f \n", i, state->time_s);

            state->foot[i].swing_start_W = state->foot[i].contact_W; //Update swing start foot location 

            //Find the lift-off pose if this update arrived just after lift-off
            double time_since_liftoff_s = state->leg_phase[i].cycle_time_s - gait->stance_fraction * gait->period_s;
            BodyPose2D body_at_liftoff = body_pose_update(
                state->body_pose,
                forward_velocity_m_s,
                yaw_rate_rad_s,
                -time_since_liftoff_s
            );

            state->foot[i].swing_end_W = foot_placement_at_liftoff(
                body_at_liftoff,
                forward_velocity_m_s,
                yaw_rate_rad_s,
                gait,
                foot_config->nominal_foot_x_G_m[i],
                foot_config->nominal_foot_y_G_m[i],
                foot_config->ground_z_W_m
            ); //update end of swing foot location

        } else if (previous_phase == GAIT_SWING && current_phase == GAIT_STANCE) //Touch down occured
        {
            printf("Leg %d - Touch down at %.3f \n", i, state->time_s);

            state->foot[i].contact_W = state->foot[i].swing_end_W; //update the position the foot must remain in contact with
        }

        //Calculate the current ball-bottom position in W
        state->foot_position_W[i] = foot_position_at_phase(
            state->leg_phase[i].phase,
            state->leg_phase[i].progress,
            state->foot[i].contact_W,
            state->foot[i].swing_start_W,
            state->foot[i].swing_end_W,
            swing_height_m
        );

        //Convert the ball bottom in W into the ball centre in H
        state->foot_center_H[i] = foot_world_to_hip(
            state->foot_position_W[i],
            transform_config->ball_radius_m,
            body_position_W_m,
            R_WB,
            transform_config->hip_position_B_m[i]
        );

        //Solve the three joint angles from this leg's hip roll-frame target
        state->joint_angles[i] = leg_inverse_kinematics(
            state->foot_center_H[i],
            ik_config->upper_link_length_m,
            ik_config->lower_link_length_m,
            ik_config->hip_roll_to_pitch_m[i],
            ik_config->upper_lateral_offset_m[i],
            ik_config->lower_lateral_offset_m[i]
        );
    }

    
}

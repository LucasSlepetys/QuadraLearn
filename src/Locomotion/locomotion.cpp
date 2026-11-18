/*

FOR EACH UPDATE:

    Advance real time by dt, and motion time by ds
    Use motion time for the body and all four leg phases

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
#include "../ForwardKinematics/forward_kinematics.h"
#include <math.h>

//Shared equations: p_W -> c_H -> {alpha, theta_1, theta_2} for the current body pose.
static void locomotion_solve_joint_angles(
    LocomotionState *state,
    double ground_z_W_m,
    const CoordinateTransformConfig *transform_config,
    const InverseKinematicsConfig *ik_config
) {
    //Current body position b_W and rotation R_WB, shared by all four legs
    double body_position_W_m[3] = {
        state->body_pose.x_W_m,
        state->body_pose.y_W_m,
        ground_z_W_m + transform_config->body_height_m
    };

    double R_WB[3][3];
    body_rotation_matrix(
        state->body_pose.yaw_rad,
        transform_config->body_pitch_rad,
        transform_config->body_roll_rad,
        R_WB
    );

    for (int i = 0; i < 4; i++) {
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

int locomotion_initialize(
    LocomotionState *state,
    BodyPose2D initial_body_pose,
    const LegJointAngles initial_joint_angles[4],
    double ground_z_W_m,
    CoordinateTransformConfig *transform_config,
    const InverseKinematicsConfig *ik_config
) {

    LegPointsHip points_H[4]; //FK points; F is the ball centre c_H
    double H_i[4]; //Body height implied by each grounded foot [m]
    double H0 = 0;
    double R_f = transform_config->ball_radius_m;
    const double height_tolerance_m = 0.0001; //0.1 mm for rounded input angles

    //R_WB maps B to W; roll/pitch are assumed known (zero for this test)
    double R_WB[3][3];
    body_rotation_matrix(initial_body_pose.yaw_rad,
                         transform_config->body_pitch_rad,
                         transform_config->body_roll_rad, R_WB);

    for (int i = 0; i < 4; i++) {
        //1. q_i,0 -> c_H using the existing leg geometry and FK
        points_H[i] = leg_forward_kinematics(
            initial_joint_angles[i],
            ik_config->upper_link_length_m,
            ik_config->lower_link_length_m,
            ik_config->hip_roll_to_pitch_m[i],
            ik_config->upper_lateral_offset_m[i],
            ik_config->lower_lateral_offset_m[i]
        );

        //2. c_B = h_B + c_H: locate the centre relative to the body origin
        double c_B[3];
        for (int coordinate = 0; coordinate < 3; coordinate++) {
            c_B[coordinate] = transform_config->hip_position_B_m[i][coordinate]
                            + points_H[i].F[coordinate];
        }

        //3. H_i = R_f - [R_WB*c_B]_z; a grounded centre is R_f above the floor
        H_i[i] = R_f - (R_WB[2][0] * c_B[0]
                     + R_WB[2][1] * c_B[1]
                     + R_WB[2][2] * c_B[2]);
        if (!isfinite(H_i[i]) || H_i[i] <= 0) return 0;
        H0 += H_i[i];
    }

    //4. H0 = mean(H_i); one body height must fit all four grounded feet
    H0 /= 4;
    for (int i = 0; i < 4; i++) {
        if (fabs(H_i[i] - H0) > height_tolerance_m) return 0;
    }

    transform_config->body_height_m = H0;
    state->time_s = 0;
    state->motion_time_s = 0; //Standing has not advanced the gait
    state->body_pose = initial_body_pose;
    double b_W[3] = {initial_body_pose.x_W_m, initial_body_pose.y_W_m, ground_z_W_m + H0};

    for (int i = 0; i < 4; i++) {
        //5. c_W = b_W + R_WB*(h_B + c_H), using the existing transform
        LegPointsWorld points_W = leg_points_hip_to_world(
            points_H[i], transform_config->hip_position_B_m[i], b_W, R_WB
        );

        //Q_W = c_W - R_f*e_z: the ball bottom becomes the saved ground contact
        WorldFootprint Q_W = {points_W.F[0], points_W.F[1], points_W.F[2] - R_f};
        state->foot[i].contact_W = Q_W;
        state->foot[i].swing_start_W = Q_W; //Placeholders until walking starts
        state->foot[i].swing_end_W = Q_W;

        WorldFootPosition p_W = {Q_W.x_W_m, Q_W.y_W_m, Q_W.z_W_m};
        HipFootPosition c_H = {points_H[i].F[0], points_H[i].F[1], points_H[i].F[2]};
        state->foot_position_W[i] = p_W;
        state->foot_center_H[i] = c_H;
        state->joint_angles[i] = initial_joint_angles[i]; //Keep the supplied angles

        GaitPhaseResult standing_phase = {0, 0, GAIT_STANCE, 0};
        state->leg_phase[i] = standing_phase; //The repeating gait has not started
    }

    return 1;
}

int locomotion_update_standing(
    LocomotionState *state,
    double dt_s,
    double ground_z_W_m,
    const CoordinateTransformConfig *transform_config,
    const InverseKinematicsConfig *ik_config
) {
    //Try the new height in a copy, so invalid IK cannot replace the last valid pose.
    LocomotionState next = *state;
    for (int i = 0; i < 4; i++) {
        if (state->leg_phase[i].phase != GAIT_STANCE) return 0;
        WorldFootprint Q = state->foot[i].contact_W;
        WorldFootPosition p_W = {Q.x_W_m, Q.y_W_m, Q.z_W_m};
        next.foot_position_W[i] = p_W; //All four world contacts stay fixed
    }

    locomotion_solve_joint_angles(&next, ground_z_W_m, transform_config, ik_config);
    for (int i = 0; i < 4; i++) {
        LegJointAngles q = next.joint_angles[i];
        if (!isfinite(q.alpha_rad) || !isfinite(q.theta_1_rad) || !isfinite(q.theta_2_rad)) return 0;
    }

    next.time_s += dt_s; //Only real time advances; x, y, yaw and motion time stay fixed
    *state = next;
    return 1;
}

void locomotion_begin_walk(
    LocomotionState *state,
    double forward_velocity_m_s,
    double yaw_rate_rad_s,
    const GaitConfig *gait,
    const FootPlacementConfig *foot_config
) {
    for (int i = 0; i < 4; i++) {
        //At s = 0, our trot selects FR/RL for swing with progress u = 0
        state->leg_phase[i] = gait_phase_at_time(
            state->motion_time_s, gait->period_s, gait->stance_fraction, gait->phase_offset_s[i]
        );

        if (state->leg_phase[i].phase == GAIT_SWING) {
            //Q_start is the actual standing contact; Q_end is the next planned landing
            state->foot[i].swing_start_W = state->foot[i].contact_W;
            state->foot[i].swing_end_W = foot_placement_at_liftoff(
                state->body_pose, forward_velocity_m_s, yaw_rate_rad_s, gait,
                foot_config->nominal_foot_x_G_m[i],
                foot_config->nominal_foot_y_G_m[i], foot_config->ground_z_W_m
            );
        }
    }
}

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
) {

    state->time_s += dt_s; //Real time always advances, including while standing
    if (motion_dt_s <= 0 && !finish_walk) return; //Keep the held pose after stopping too

    //Use nominal v0/omega0 with ds; multiplying the commands by lambda too would slow twice
    state->body_pose = body_pose_update(state->body_pose, forward_velocity_m_s, yaw_rate_rad_s, motion_dt_s);
    state->motion_time_s += motion_dt_s; //Body motion and gait share this same s
    
    //For each leg:
    for (int i = 0; i < 4; i++) {

        // Previous phrase before update:
        Phase previous_phase = state->leg_phase[i].phase;

        // Update cycle_index, cycle_time_s, phase, progress for each leg
        state->leg_phase[i] = gait_phase_at_time(state->motion_time_s, gait->period_s, gait->stance_fraction, gait->phase_offset_s[i]);
        
        // Phase after the update:
        Phase current_phase = state->leg_phase[i].phase;

        if (finish_walk) {
            //At s_b, land the incoming pair and suppress the outgoing pair's lift-off.
            if (previous_phase == GAIT_SWING) {
                state->foot[i].contact_W = state->foot[i].swing_end_W;
                printf("Leg %d - Final touchdown at %.3f \n", i, state->time_s);
            }
            state->leg_phase[i].phase = GAIT_STANCE; //Actual held state, not a continuing trot
            state->leg_phase[i].progress = 0;
        } else if(previous_phase == GAIT_STANCE && current_phase == GAIT_SWING) //Lift off occured
        {
            printf("Leg %d - Lift off at %.3f \n", i, state->time_s);

            state->foot[i].swing_start_W = state->foot[i].contact_W; //Update swing start foot location 

            //Rewind by motion time since lift-off to predict from the actual event pose
            double motion_time_since_liftoff_s = state->leg_phase[i].cycle_time_s - gait->stance_fraction * gait->period_s;
            BodyPose2D body_at_liftoff = body_pose_update(
                state->body_pose,
                forward_velocity_m_s,
                yaw_rate_rad_s,
                -motion_time_since_liftoff_s
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
    }

    locomotion_solve_joint_angles(state, foot_config->ground_z_W_m, transform_config, ik_config);
}

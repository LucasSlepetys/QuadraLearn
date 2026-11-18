#include "simulation_export.h"
#include "../ForwardKinematics/forward_kinematics.h"

FILE *simulation_export_open(const char *file_path)
{
    FILE *file = fopen(file_path, "w");
    if (file == NULL) {
        return NULL;
    }

    //Shared data; body position is b_W, angles define R_WB
    fprintf(file, "time_s,body_x_W_m,body_y_W_m,body_z_W_m"
                  ",body_yaw_rad,body_pitch_rad,body_roll_rad"
                  ",ground_z_W_m,ball_radius_m");

    const char *leg_names[4] = {"FL", "FR", "RL", "RR"};
    const char *foot_names[4] = {"contact", "swing_start", "swing_end", "foot_bottom"};
    const char *point_names[4] = {"H", "P", "K", "F"};

    for (int i = 0; i < 4; i++) {
        const char *leg = leg_names[i];

        //Gait timing and the three mathematical joint angles
        fprintf(file, ",%s_cycle_index,%s_cycle_time_s,%s_phase,%s_progress", leg, leg, leg, leg);
        fprintf(file, ",%s_alpha_rad,%s_theta_1_rad,%s_theta_2_rad", leg, leg, leg);

        //Remembered footprints and the current ball-bottom target, all in W
        for (int point = 0; point < 4; point++) {
            fprintf(file, ",%s_%s_x_W_m,%s_%s_y_W_m,%s_%s_z_W_m",
                    leg, foot_names[point], leg, foot_names[point], leg, foot_names[point]);
        }

        //Ball-centre target in the fixed hip-roll frame H
        fprintf(file, ",%s_foot_center_x_H_m,%s_foot_center_y_H_m,%s_foot_center_z_H_m", leg, leg, leg);

        //Reconstructed H, P, K, F points, each expressed in W
        for (int point = 0; point < 4; point++) {
            fprintf(file, ",%s_%s_x_W_m,%s_%s_y_W_m,%s_%s_z_W_m",
                    leg, point_names[point], leg, point_names[point], leg, point_names[point]);
        }
    }
    fprintf(file, "\n");

    if (ferror(file)) {
        fclose(file);
        return NULL;
    }
    return file;
}

void simulation_export_write(
    FILE *file,
    const LocomotionState *state,
    double ground_z_W_m,
    const CoordinateTransformConfig *transform_config,
    const InverseKinematicsConfig *ik_config
) {

    //1. Current body origin: b_W = {x_B, y_B, z_ground + H}
    double body_position_W_m[3] = {
        state->body_pose.x_W_m,
        state->body_pose.y_W_m,
        ground_z_W_m + transform_config->body_height_m
    };

    //R_WB = R_z(yaw) * R_y(pitch) * R_x(roll)
    double R_WB[3][3];
    body_rotation_matrix(
        state->body_pose.yaw_rad,
        transform_config->body_pitch_rad,
        transform_config->body_roll_rad,
        R_WB
    );

    //2. Start this row with time, body pose, ground height and ball radius
    //17 significant digits preserve double precision when Python reads the file
    fprintf(file, "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g",
            state->time_s,
            body_position_W_m[0], body_position_W_m[1], body_position_W_m[2],
            state->body_pose.yaw_rad,
            transform_config->body_pitch_rad, transform_config->body_roll_rad,
            ground_z_W_m, transform_config->ball_radius_m);

    for (int i = 0; i < 4; i++) {

        //3. Joint angles -> H, P, K, F in the fixed hip-roll frame
        LegPointsHip points_H = leg_forward_kinematics(
            state->joint_angles[i],
            ik_config->upper_link_length_m,
            ik_config->lower_link_length_m,
            ik_config->hip_roll_to_pitch_m[i],
            ik_config->upper_lateral_offset_m[i],
            ik_config->lower_lateral_offset_m[i]
        );

        //4. Each point: X_W = b_W + R_WB * (h_B + X_H)
        LegPointsWorld points_W = leg_points_hip_to_world(
            points_H,
            transform_config->hip_position_B_m[i],
            body_position_W_m,
            R_WB
        );

        //5. Write this leg's values in the same order as the column names
        fprintf(file, ",%d,%.17g,%s,%.17g",
                state->leg_phase[i].cycle_index,
                state->leg_phase[i].cycle_time_s,
                (state->leg_phase[i].phase == GAIT_STANCE) ? "STANCE" : "SWING",
                state->leg_phase[i].progress);

        fprintf(file, ",%.17g,%.17g,%.17g",
                state->joint_angles[i].alpha_rad,
                state->joint_angles[i].theta_1_rad,
                state->joint_angles[i].theta_2_rad);

        //Ground-contact memory: Q_contact, Q_start, Q_end in W
        fprintf(file, ",%.17g,%.17g,%.17g",
                state->foot[i].contact_W.x_W_m,
                state->foot[i].contact_W.y_W_m,
                state->foot[i].contact_W.z_W_m);
        fprintf(file, ",%.17g,%.17g,%.17g",
                state->foot[i].swing_start_W.x_W_m,
                state->foot[i].swing_start_W.y_W_m,
                state->foot[i].swing_start_W.z_W_m);
        fprintf(file, ",%.17g,%.17g,%.17g",
                state->foot[i].swing_end_W.x_W_m,
                state->foot[i].swing_end_W.y_W_m,
                state->foot[i].swing_end_W.z_W_m);

        //Targets: ball bottom p_W, then ball centre c_H used by IK
        fprintf(file, ",%.17g,%.17g,%.17g",
                state->foot_position_W[i].x_W_m,
                state->foot_position_W[i].y_W_m,
                state->foot_position_W[i].z_W_m);
        fprintf(file, ",%.17g,%.17g,%.17g",
                state->foot_center_H[i].x_H_m,
                state->foot_center_H[i].y_H_m,
                state->foot_center_H[i].z_H_m);

        //FK drawing points in W; F is the ball centre
        fprintf(file, ",%.17g,%.17g,%.17g", points_W.H[0], points_W.H[1], points_W.H[2]);
        fprintf(file, ",%.17g,%.17g,%.17g", points_W.P[0], points_W.P[1], points_W.P[2]);
        fprintf(file, ",%.17g,%.17g,%.17g", points_W.K[0], points_W.K[1], points_W.K[2]);
        fprintf(file, ",%.17g,%.17g,%.17g", points_W.F[0], points_W.F[1], points_W.F[2]);
    }
    fprintf(file, "\n"); //All four legs belong to this same instant
}

int simulation_export_close(FILE *file)
{
    int write_failed = ferror(file);
    int close_result = fclose(file); //Flush the remaining data and close
    return (write_failed || close_result != 0) ? EOF : 0;
}

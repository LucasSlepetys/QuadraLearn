#include <stdio.h>
#include <math.h>
#include "Locomotion/locomotion.h"
#include "Configuration/robot_model.h"
#include "Configuration/chosen_configurations.h"
#include "SimulationExport/simulation_export.h"
#include "MotionSupervisor/motion_supervisor.h"

const double radians_to_degrees = 180.0 / 3.141592653589793; //Display only

MotionCommand command = initial_command;
FootPlacementConfig foot_config = make_foot_placement_config(x_0_m, y_0_m, ground_z_W_m);
CoordinateTransformConfig transform_config = make_coordinate_transform_config(&command);

LocomotionState state = {}; //Inicialize every state varaible to zero
MotionSupervisorState supervisor = {}; //Starts in MOTION_STAND with no requests

int main(void)
{
    
    if (!isfinite(standing_delay_s) || standing_delay_s < 0) {
        fprintf(stderr, "Standing delay must be finite and nonnegative.\n");
        return 1;
    }

    if (!isfinite(stop_request_time_s) || stop_request_time_s < standing_delay_s) {
        fprintf(stderr, "Desktop STOP time must be at or after height preparation begins.\n");
        return 1;
    }

    //q_i,0 -> FK -> common body height H0 and four fixed ground contacts
    if (!locomotion_initialize(
        &state,
        initial_body_pose,
        initial_joint_angles,
        ground_z_W_m,
        &transform_config,
        &ik_config
    )) {
        fprintf(stderr, "Starting angles do not form a consistent grounded pose for the chosen body orientation.\n");
        return 1;
    }

    //Keep the request H_target separate from the current height H0 recovered by FK.
    printf("Standing body height H0: %.6f m | target height: %.6f m\n",
           transform_config.body_height_m, command.body_height_m);


    //Record the initial state at t = 0; VS Code saves this in the project folder
    FILE *simulation_file = simulation_export_open("simulation.csv");
    if (simulation_file == NULL) {
        perror("Could not create simulation.csv");
        return 1;
    }
    simulation_export_write(simulation_file, &state, foot_config.ground_z_W_m, &transform_config, &ik_config);

    //Check the front-right footprints before the update loop
    printf("Initial FR footprints at t = %.3f s\n", state.time_s);
    printf("Contact:     (%.9f, %.9f, %.9f) m\n",
        state.foot[1].contact_W.x_W_m, state.foot[1].contact_W.y_W_m, state.foot[1].contact_W.z_W_m);
    printf("Swing start: (%.9f, %.9f, %.9f) m\n",
        state.foot[1].swing_start_W.x_W_m, state.foot[1].swing_start_W.y_W_m, state.foot[1].swing_start_W.z_W_m);
    printf("Swing end:   (%.9f, %.9f, %.9f) m\n",
        state.foot[1].swing_end_W.x_W_m, state.foot[1].swing_end_W.y_W_m, state.foot[1].swing_end_W.z_W_m);

    int preparation_started = 0;
    int start_sent = 0; //Send each desktop request once; do not restart after stopping
    int stop_sent = 0;
    for(int i = 0; i < simulation_updates; i++) {
        double update_dt_s = dt_s;
        do {
            if (!preparation_started && state.time_s >= standing_delay_s - 1e-12) {
                if (!motion_supervisor_begin_height_adjustment(
                    &supervisor, transform_config.body_height_m,
                    command.body_height_m, height_adjustment_duration_s
                )) {
                    fprintf(stderr, "Height targets and adjustment duration must be finite and positive.\n");
                    simulation_export_close(simulation_file);
                    return 1;
                }
                preparation_started = 1;
                printf("HEIGHT: real t = %.3f s, %.6f -> %.6f m\n",
                       state.time_s, supervisor.height_start_m, supervisor.height_target_m);
            }

            if (!stop_sent && state.time_s >= stop_request_time_s - 1e-12) {
                if (!motion_supervisor_request_stop(&supervisor, minimum_stop_duration_s, &gait)) {
                    fprintf(stderr, "STOP needs a positive duration and the configured 50%% diagonal trot.\n");
                    simulation_export_close(simulation_file);
                    return 1;
                }
                stop_sent = 1;
                printf("STOP requested: real t = %.3f s, motion s = %.6f s\n", state.time_s, state.motion_time_s);
            }

            //Only begin walking after reaching H_target; STOP during preparation cancels this start.
            if (preparation_started && !start_sent && !stop_sent && supervisor.mode == MOTION_STAND) {
                if (!motion_supervisor_start(&supervisor, start_ramp_duration_s)) {
                    fprintf(stderr, "Start-ramp duration must be finite and positive.\n");
                    simulation_export_close(simulation_file);
                    return 1;
                }
                start_sent = 1;
                locomotion_begin_walk(&state, command.forward_velocity_m_s,
                                      command.yaw_rate_rad_s, &gait, &foot_config);
                printf("START: real t = %.3f s, motion s = %.6f s\n", state.time_s, state.motion_time_s);
            }

            //Split at preparation, height completion or STOP; keep the remaining real time.
            double interval_s = update_dt_s;
            if (!preparation_started) interval_s = fmin(interval_s, standing_delay_s - state.time_s);
            if (!stop_sent) interval_s = fmin(interval_s, stop_request_time_s - state.time_s);
            if (supervisor.mode == MOTION_ADJUSTING_HEIGHT) {
                interval_s = fmin(interval_s, supervisor.height_duration_s - supervisor.height_elapsed_s);
            }

            MotionMode previous_mode = supervisor.mode;
            double motion_dt_s = motion_supervisor_update(&supervisor, interval_s, state.motion_time_s);
            int finish_walk = previous_mode != MOTION_STAND
                           && previous_mode != MOTION_ADJUSTING_HEIGHT && supervisor.mode == MOTION_STAND;

            if (previous_mode == MOTION_ADJUSTING_HEIGHT) {
                double previous_height_m = transform_config.body_height_m;
                transform_config.body_height_m = supervisor.body_height_m; //H(t), not the final request
                if (!locomotion_update_standing(&state, interval_s, foot_config.ground_z_W_m,
                                                &transform_config, &ik_config)) {
                    transform_config.body_height_m = previous_height_m;
                    fprintf(stderr, "Height adjustment left the reachable leg workspace; last valid pose kept.\n");
                    simulation_export_close(simulation_file);
                    return 1;
                }
            } else {
                //Body and gait use the same ds; while standing this simply advances real time.
                locomotion_update(&state, command.forward_velocity_m_s, command.yaw_rate_rad_s,
                                  interval_s, motion_dt_s, finish_walk, &gait, &foot_config, swing_height_m,
                                  &transform_config, &ik_config);
            }
            update_dt_s -= interval_s;

            if (previous_mode == MOTION_ADJUSTING_HEIGHT && supervisor.mode == MOTION_STAND) {
                printf("HEIGHT READY: real t = %.3f s, H = %.6f m\n", state.time_s, transform_config.body_height_m);
            }
            if (previous_mode == MOTION_STARTING && supervisor.mode == MOTION_WALKING) {
                printf("WALKING: real t = %.3f s, motion s = %.6f s, rate = %.3f\n",
                       state.time_s, state.motion_time_s, supervisor.motion_rate);
            }
            if (previous_mode != MOTION_STOPPING && supervisor.mode == MOTION_STOPPING) {
                printf("STOPPING: T_d = %.6f real s, final landing s_b = %.6f motion s\n",
                       supervisor.stop_duration_s, supervisor.stop_target_motion_s);
            }
            if (finish_walk) {
                printf("STAND: real t = %.3f s, motion s = %.6f s; four feet held\n",
                       state.time_s, state.motion_time_s);
            }
        } while (update_dt_s > 0);

        //Append this instant: body pose, leg states, targets, angles and FK points
        simulation_export_write(simulation_file, &state, foot_config.ground_z_W_m, &transform_config, &ik_config);
    }

    //Finish the CSV recording before displaying the usual results
    if (simulation_export_close(simulation_file) != 0) {
        fprintf(stderr, "Could not finish writing simulation.csv\n");
        return 1;
    }

    printf("Time:   %.3f s\n", state.time_s);
    printf("Motion time s: %.6f s | rate lambda: %.3f\n", state.motion_time_s, supervisor.motion_rate);
    printf("Body x: %.5f m\n", state.body_pose.x_W_m);
    printf("Body y: %.5f m\n", state.body_pose.y_W_m);
    printf("Body yaw: %.5f rad\n", state.body_pose.yaw_rad);

    for(int i = 0; i < 4; i++) {
        printf("Result %d -> n: %d | tau: %.3f s | phase: %s | u: %.3f\n",
                i,
                state.leg_phase[i].cycle_index,
                state.leg_phase[i].cycle_time_s,
                (state.leg_phase[i].phase == GAIT_STANCE) ? "GAIT_STANCE" : "GAIT_SWING",
                state.leg_phase[i].progress);

        printf("Foot %d in W: (%.5f, %.5f, %.5f) m\n",
                i,
                state.foot_position_W[i].x_W_m,
                state.foot_position_W[i].y_W_m,
                state.foot_position_W[i].z_W_m);

        printf("Ball centre %d in H: (%.5f, %.5f, %.5f) m\n",
                i,
                state.foot_center_H[i].x_H_m,
                state.foot_center_H[i].y_H_m,
                state.foot_center_H[i].z_H_m);

        printf("Angles %d: alpha = %.2f | theta_1 = %.2f | theta_2 = %.2f deg\n",
                i,
                state.joint_angles[i].alpha_rad * radians_to_degrees,
                state.joint_angles[i].theta_1_rad * radians_to_degrees,
                state.joint_angles[i].theta_2_rad * radians_to_degrees);
    }
    

    

    

    return 0;
}
#include <stdio.h>
#include "Locomotion/locomotion.h"
#include "Configuration/robot_model.h"
#include "Configuration/chosen_configurations.h"
#include "SimulationExport/simulation_export.h"

const double radians_to_degrees = 180.0 / 3.141592653589793; //Display only

MotionCommand command = initial_command;
FootPlacementConfig foot_config = make_foot_placement_config(x_0_m, y_0_m, ground_z_W_m);
CoordinateTransformConfig transform_config = make_coordinate_transform_config(&command);

LocomotionState state = {}; //Inicialize every state varaible to zero

int main(void)
{
    
    locomotion_initialize(
        &state,
        initial_body_pose,
        command.forward_velocity_m_s,
        command.yaw_rate_rad_s,
        &gait,
        &foot_config,
        swing_height_m,
        &transform_config,
        &ik_config
    );

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

    for(int i = 0; i < simulation_updates; i++) {
        // A future upstream module can replace command here before this update
        transform_config = make_coordinate_transform_config(&command);

        locomotion_update(&state, command.forward_velocity_m_s, command.yaw_rate_rad_s, dt_s, &gait, &foot_config, swing_height_m, &transform_config, &ik_config);

        //Append this instant: body pose, leg states, targets, angles and FK points
        simulation_export_write(simulation_file, &state, foot_config.ground_z_W_m, &transform_config, &ik_config);
    }

    //Finish the CSV recording before displaying the usual results
    if (simulation_export_close(simulation_file) != 0) {
        fprintf(stderr, "Could not finish writing simulation.csv\n");
        return 1;
    }

    printf("Time:   %.3f s\n", state.time_s);
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
#ifndef SIMULATION_EXPORT
#define SIMULATION_EXPORT

#include <stdio.h>
#include "../Locomotion/locomotion.h"

//One row per instant: time/body data, then FL, FR, RL, RR
//Positions [m], angles [rad], phase is STANCE or SWING, progress is dimensionless
//H/P/K/F are FK points in W: hip roll, hip pitch, knee, ball centre
//foot_bottom is the target in W; foot_center is the IK target in H
//contact/swing_start/swing_end are remembered ball-bottom footprints in W

//Create the file and column names; replaces a previous recording at this path
FILE *simulation_export_open(const char *file_path);

//Read the current state, calculate FK points, and append one complete row
void simulation_export_write(
    FILE *file,
    const LocomotionState *state,
    double ground_z_W_m,
    const CoordinateTransformConfig *transform_config,
    const InverseKinematicsConfig *ik_config
);

//Finish writing and close the file; returns 0 on success, EOF on a file error
int simulation_export_close(FILE *file);

#endif // SIMULATION_EXPORT

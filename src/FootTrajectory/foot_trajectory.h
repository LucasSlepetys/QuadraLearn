#ifndef FOOT_TRAJECTORY
#define FOOT_TRAJECTORY

#include <stdio.h>
#include "../FootPlacement/foot_placement.h"

/*
WorldFootPosition is not the same as WorldFootPrint
Of bottom of foot

double x_W_m;
double y_W_m;
double z_W_m;
*/
typedef struct {
    double x_W_m;
    double y_W_m;
    double z_W_m;
} WorldFootPosition; //P_W changing over time

WorldFootPosition foot_position_at_phase(
    Phase phase,
    double progress,
    WorldFootprint contact_W, //Fixed ground position during stance
    WorldFootprint swing_start_W, //Saved lift off position
    WorldFootprint swing_end_W, //Saved touchdown destination
    double swing_height_m //Max ball-bottom clearance above ground (h)
);

#endif // FOOT_TRAJECTORY
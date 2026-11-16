#include "foot_trajectory.h"

WorldFootPosition foot_position_at_phase(
    Phase phase,
    double progress,
    WorldFootprint contact_W,
    WorldFootprint swing_start_W,
    WorldFootprint swing_end_W,
    double swing_height_m
) {

    WorldFootPosition foot_position;

    if(phase == GAIT_STANCE) {
        //Keep the foot at its fixed world contact
        foot_position.x_W_m = contact_W.x_W_m;
        foot_position.y_W_m = contact_W.y_W_m;
        foot_position.z_W_m = contact_W.z_W_m;
    } else {
        double u = progress; //Swing progress from 0 to 1
        double u2 = u * u;
        double u3 = u2 * u;
        double u4 = u3 * u;
        double u5 = u4 * u;

        double F = 10 * u3 - 15 * u4 + 6 * u5; //Progress between endpoints
        double Z = 64 * u3 * (1 - u) * (1 - u) * (1 - u); //Lift shape

        //Move between the endpoints and add lift along world z with the two smooth functions
        foot_position.x_W_m = swing_start_W.x_W_m + F * (swing_end_W.x_W_m - swing_start_W.x_W_m);
        foot_position.y_W_m = swing_start_W.y_W_m + F * (swing_end_W.y_W_m - swing_start_W.y_W_m);
        foot_position.z_W_m = swing_start_W.z_W_m + F * (swing_end_W.z_W_m - swing_start_W.z_W_m) + swing_height_m * Z;
    }

    return foot_position;
}

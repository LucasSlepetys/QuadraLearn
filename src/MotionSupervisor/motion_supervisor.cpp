#include "motion_supervisor.h"
#include <math.h>

//F(u) changes smoothly from 0 to 1; use 1-F(u) when slowing down.
static double F(double u) {
    double u2 = u * u;
    double u3 = u2 * u;
    return 10 * u3 - 15 * u3 * u + 6 * u3 * u2;
}

//G(u) is accumulated motion progress: its derivative is the smooth rate F(u).
static double G(double u) {
    double u2 = u * u;
    double u4 = u2 * u2;
    return 2.5 * u4 - 3 * u4 * u + u4 * u2;
}

int motion_supervisor_begin_height_adjustment(
    MotionSupervisorState *state,
    double initial_height_m,
    double target_height_m,
    double duration_s
) {
    if (!isfinite(initial_height_m) || initial_height_m <= 0) return 0;
    if (!isfinite(target_height_m) || target_height_m <= 0) return 0;
    if (!isfinite(duration_s) || duration_s <= 0) return 0;

    state->height_start_m = initial_height_m;
    state->height_target_m = target_height_m;
    state->body_height_m = initial_height_m;
    state->height_elapsed_s = 0;
    state->height_duration_s = duration_s;
    state->motion_rate = 0;
    state->stop_requested = 0;
    state->mode = fabs(target_height_m - initial_height_m) < 1e-12
                ? MOTION_STAND : MOTION_ADJUSTING_HEIGHT;
    return 1;
}

int motion_supervisor_start(MotionSupervisorState *state, double start_ramp_duration_s) {
    if (!isfinite(start_ramp_duration_s) || start_ramp_duration_s <= 0) return 0;

    state->mode = MOTION_STARTING;
    state->start_elapsed_s = 0;
    state->start_ramp_duration_s = start_ramp_duration_s;
    state->motion_rate = 0;
    state->stop_requested = 0;
    return 1;
}

int motion_supervisor_request_stop(
    MotionSupervisorState *state,
    double minimum_stop_duration_s,
    const GaitConfig *gait
) {
    if (state->mode == MOTION_STAND || state->mode == MOTION_STOPPING || state->stop_requested) return 1;
    if (state->mode == MOTION_ADJUSTING_HEIGHT) {
        state->stop_requested = 1; //Finish lowering/raising, then keep standing
        return 1;
    }
    if (!isfinite(minimum_stop_duration_s) || minimum_stop_duration_s <= 0) return 0;
    if (!isfinite(gait->period_s) || gait->period_s <= 0) return 0;
    if (gait->stance_fraction != 0.5) return 0;

    //The landing rule below assumes these exact diagonal-pair offsets.
    const double offsets[4] = {0, gait->period_s / 2, gait->period_s / 2, 0};
    for (int i = 0; i < 4; i++) {
        if (gait->phase_offset_s[i] != offsets[i]) return 0;
    }

    state->stop_requested = 1;
    state->minimum_stop_duration_s = minimum_stop_duration_s;
    state->landing_interval_s = gait->period_s / 2;
    return 1;
}

double motion_supervisor_update(MotionSupervisorState *state, double dt_s, double motion_time_s) {
    if (state->mode == MOTION_STAND || dt_s <= 0) return 0;
    if (state->mode == MOTION_ADJUSTING_HEIGHT) {
        double T_H = state->height_duration_s;
        state->height_elapsed_s = fmin(state->height_elapsed_s + dt_s, T_H);
        double u = state->height_elapsed_s / T_H;

        //H(t) = H0 + (H_target - H0)*F(u); feet stay on their saved contacts
        state->body_height_m = state->height_start_m
                             + (state->height_target_m - state->height_start_m) * F(u);
        if (state->height_elapsed_s >= T_H - 1e-12) {
            state->height_elapsed_s = T_H;
            state->body_height_m = state->height_target_m;
            state->mode = MOTION_STAND; //Main can now request the walking start
        }
        return 0; //Real time advances, but the walking clock s stays paused
    }

    double motion_dt_s = 0;

    if (state->mode == MOTION_STARTING) {
        double T_a = state->start_ramp_duration_s;
        double remaining_s = T_a - state->start_elapsed_s;
        double ramp_dt_s = fmin(dt_s, remaining_s);
        double u_old = state->start_elapsed_s / T_a;
        state->start_elapsed_s += ramp_dt_s;
        double u = state->start_elapsed_s / T_a;

        //s - s_a = T_a*G(u); subtract consecutive values to get this update's ds
        motion_dt_s = T_a * (G(u) - G(u_old));
        state->motion_rate = F(u);
        dt_s -= ramp_dt_s; //Keep any real time left after the start ramp
        if (ramp_dt_s < remaining_s) return motion_dt_s;

        state->motion_rate = 1;
        state->mode = MOTION_WALKING;
    }

    if (state->mode == MOTION_WALKING) {
        if (!state->stop_requested) return motion_dt_s + dt_s; //ds = dt at normal rate

        //s_b = L*ceil((s_d + T_min/2)/L), then T_d = 2*(s_b - s_d)
        double L = state->landing_interval_s;
        double s_d = motion_time_s + motion_dt_s;
        double boundary_number = (s_d + state->minimum_stop_duration_s / 2) / L;
        //Avoid choosing an extra half-cycle for round-off at an exact boundary.
        double s_b = L * ceil(boundary_number - 1e-12);
        state->stop_start_motion_s = s_d;
        state->stop_target_motion_s = s_b;
        state->stop_duration_s = 2 * (s_b - s_d);
        state->stop_elapsed_s = 0;
        state->stop_requested = 0;
        state->mode = MOTION_STOPPING;
    }

    double T_d = state->stop_duration_s;
    state->stop_elapsed_s = fmin(state->stop_elapsed_s + dt_s, T_d);
    double u_d = state->stop_elapsed_s / T_d;

    //lambda = 1-F(u_d); s = s_d + T_d*(u_d - G(u_d))
    state->motion_rate = 1 - F(u_d);
    double s = state->stop_start_motion_s + T_d * (u_d - G(u_d));

    if (state->stop_elapsed_s >= T_d - 1e-12 || s >= state->stop_target_motion_s) {
        //Snap to the chosen landing; any leftover real time is standing time.
        s = state->stop_target_motion_s;
        state->stop_elapsed_s = T_d;
        state->motion_rate = 0;
        state->mode = MOTION_STAND;
    }

    return s - motion_time_s; //Includes any start-ramp remainder in this same update
}

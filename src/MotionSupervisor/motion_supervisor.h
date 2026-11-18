#ifndef MOTION_SUPERVISOR
#define MOTION_SUPERVISOR

#include "../Gait/gait.h"

typedef enum {
    MOTION_STAND,
    MOTION_ADJUSTING_HEIGHT,
    MOTION_STARTING,
    MOTION_WALKING,
    MOTION_STOPPING
} MotionMode;

//The supervisor remembers the ramps, not the robot's geometry or foot contacts.
typedef struct {
    MotionMode mode;
    double start_elapsed_s;       //Real time since START
    double start_ramp_duration_s; //T_a: chosen real duration of the ramp
    double motion_rate;          //lambda = ds/dt; 0 holds, 1 is normal walking

    double height_start_m;       //H0: saved height at the start of this adjustment
    double height_target_m;      //Chosen height to reach before walking
    double body_height_m;        //H(t): current planned height above ground
    double height_elapsed_s;     //Real time since height adjustment began
    double height_duration_s;    //T_H: chosen real duration

    int stop_requested;             //Remember STOP during height preparation or the start ramp
    double minimum_stop_duration_s; //T_min: requested minimum real stopping time
    double landing_interval_s;      //L = T/2 for our 50% diagonal trot
    double stop_elapsed_s;          //Real time since deceleration began
    double stop_duration_s;         //T_d: actual real duration, ending at a landing
    double stop_start_motion_s;     //s_d: motion time when deceleration began
    double stop_target_motion_s;    //s_b: motion time of the final landing
} MotionSupervisorState;

//Prepare H0 -> H_target with all four feet grounded. Equal heights skip this stage.
//Returns 0 for invalid heights/duration; otherwise 1. Completion returns to STAND.
int motion_supervisor_begin_height_adjustment(
    MotionSupervisorState *state,
    double initial_height_m,
    double target_height_m,
    double duration_s
);

//Begin a ramp from rest. Returns 0 for an invalid T_a; otherwise returns 1.
int motion_supervisor_start(MotionSupervisorState *state, double start_ramp_duration_s);

//During height adjustment, STOP lets it finish and prevents the automatic walk.
//During STARTING, a normal stop waits for rate 1.
//Returns 0 for invalid settings or a gait other than our 50% diagonal trot.
int motion_supervisor_request_stop(
    MotionSupervisorState *state,
    double minimum_stop_duration_s,
    const GaitConfig *gait
);

//Advance by real dt; return ds for body and gait, using their current motion time s.
//Height adjustment returns ds = 0 and updates body_height_m instead.
//Nominal forward speed/yaw rate stay fixed during walking.
double motion_supervisor_update(MotionSupervisorState *state, double dt_s, double motion_time_s);

#endif // MOTION_SUPERVISOR

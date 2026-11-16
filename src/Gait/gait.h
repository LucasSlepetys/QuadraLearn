#ifndef GAIT
#define GAIT

#include <stdio.h>

//Shared gaits:

typedef struct {
    double period_s;
    double stance_fraction;
    double phase_offset_s[4];
} GaitConfig;

/*
0 = FL
1 = FR
2 = RL
3 = RR
*/

//----

typedef enum {
    GAIT_STANCE,
    GAIT_SWING,
} Phase;

/* 
int cycle_index; //leg cycle number n
double cycle_time_s; //leg cyle time tau
Phase phase; //enum of GAIT_STANCE AND GAIT_SWING
double progress; // fraction of the current phase for GAIT and SWING (u)
*/
typedef struct 
{
    int cycle_index; //leg cycle number n
    double cycle_time_s; //leg cyle time tau
    Phase phase; //enum of GAIT_STANCE AND GAIT_SWING
    double progress; // fraction of the current phase for GAIT and SWING (u)
} GaitPhaseResult;

/* 
    Calculates gait phase results for a leg at a time given configured settings:

    double time_s,
    double gait_period_s,
    double stance_fraction,
    double phase_offset_s
*/
GaitPhaseResult gait_phase_at_time(
    double time_s,
    double gait_period_s,
    double stance_fraction,
    double phase_offset_s
);

#endif // GAIT
/*

Calculate the phase durations:
    stance_duration = stance_fraction × gait_period
    swing_duration  = (1 − stance_fraction) × gait_period

Calculate the cycle number:
    cycle_index = floor((time + phase_offset) / gait_period)

Calculate the local time within that cycle:
    cycle_time = time + phase_offset − cycle_index × gait_period

If cycle_time < stance_duration:
    phase = STANCE
    progress = cycle_time / stance_duration

Otherwise:
    phase = SWING
    progress = (cycle_time − stance_duration) / swing_duration

Return cycle_index, cycle_time, phase and progress.

*/

#include "gait.h"
#include <math.h>

GaitPhaseResult gait_phase_at_time(
    double time_s,
    double gait_period_s,
    double stance_fraction,
    double phase_offset_s
) {

    double stance_duration = stance_fraction * gait_period_s;
    double swing_duration  = (1 - stance_fraction) * gait_period_s;

    int cycle_index = floor((time_s + phase_offset_s)/gait_period_s); //cycle_index
    double cycle_time_s = time_s + phase_offset_s - cycle_index * gait_period_s; //local time tau
    Phase phase = (cycle_time_s >= 0 && cycle_time_s < stance_duration) ? GAIT_STANCE : GAIT_SWING;
    double progress = (phase == GAIT_STANCE) ? (cycle_time_s / stance_duration) : ((cycle_time_s - stance_duration) / swing_duration); // u for both stance and swing

    GaitPhaseResult result = {
        cycle_index,
        cycle_time_s,
        phase,
        progress
    };

    return result; //GaitPhraseResult
 
}
/*

Calculate stance and swing durations from the gait settings.

Calculate the prediction interval:
    swing duration + half the stance duration

Call body_pose_update:
    start from body_at_liftoff
    use the supplied velocity and yaw rate
    predict over that interval

Keep the predicted pose in a local variable.

Use its position and yaw to calculate the world footprint.

Return the three footprint coordinates.

*/

#include "foot_placement.h"
#include <math.h>

WorldFootprint foot_placement_at_liftoff(
    BodyPose2D body_at_liftoff,
    double forward_velocity_m_s,
    double yaw_rate_rad_s,
    const GaitConfig *gait,
    double nominal_foot_x_G_m,
    double nominal_foot_y_G_m,
    double ground_z_W_m
) {

    double stance_duration = gait->stance_fraction * gait->period_s;
    double swing_duration = (1 - gait->stance_fraction) * gait->period_s;
    double prediction_interval_s = swing_duration + stance_duration / 2; //!Time to next stance midpoint from lift off

    //Predict the midpoint pose without changing the current body pose
    BodyPose2D predicted_midstance_pose = body_pose_update(
        body_at_liftoff,
        forward_velocity_m_s,
        yaw_rate_rad_s,
        prediction_interval_s
    );

    return footprint_from_midstance_pose(
        predicted_midstance_pose,
        nominal_foot_x_G_m,
        nominal_foot_y_G_m,
        ground_z_W_m
    );
}

WorldFootprint footprint_from_midstance_pose(
    BodyPose2D midstance_pose,
    double nominal_foot_x_G_m,
    double nominal_foot_y_G_m,
    double ground_z_W_m
) {

    WorldFootprint next_footprint;

    double cos_yaw = cos(midstance_pose.yaw_rad);
    double sin_yaw = sin(midstance_pose.yaw_rad);

    //Rotate the nominal placement from G to W, then add the predicted body position
    next_footprint.x_W_m = midstance_pose.x_W_m + cos_yaw * nominal_foot_x_G_m - sin_yaw * nominal_foot_y_G_m;
    next_footprint.y_W_m = midstance_pose.y_W_m + sin_yaw * nominal_foot_x_G_m + cos_yaw * nominal_foot_y_G_m;
    next_footprint.z_W_m = ground_z_W_m; //Ball bottom is on the ground during stance

    return next_footprint;
}

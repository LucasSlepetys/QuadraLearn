#ifndef BODY_MOTION
#define BODY_MOTION

#include <stdio.h>

typedef struct 
{
    double x_W_m;
    double y_W_m;
    double yaw_rad; //How much it has turned
} BodyPose2D; //Where the robot is in 2d in world frame

// Function: Previous pose + motion command + elapsed time => updated pose
// Receives pose at k, returns pose at k + 1
BodyPose2D body_pose_update(
    BodyPose2D previous, 
    double forward_velocity_m_s,
    double yaw_rate_rad_s,
    double dt_s //Time between k and k+1 where speed and yaw is held constant
);

 
#endif // BODY_MOTION
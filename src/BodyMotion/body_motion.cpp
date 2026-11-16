#include "body_motion.h"
#include <math.h>


BodyPose2D body_pose_update(
    BodyPose2D previous, 
    double forward_velocity_m_s,
    double yaw_rate_rad_s,
    double dt_s
) {

    BodyPose2D new_pose;

    //Calculate half the heading change: a
    double half_heading_angle = (yaw_rate_rad_s * dt_s)/2;

    //Calculate the travel chord length (a.k.a v * dt * sinc(a))

    //sinc(a)
    double sinc = 1;
    if(half_heading_angle != 0) {
         sinc = sin(half_heading_angle) / half_heading_angle;
    } //else keep it at 1

    double travel_chord_length = forward_velocity_m_s * dt_s * sinc;
    
    //Calculate the new world x coordinate
    new_pose.x_W_m = previous.x_W_m + travel_chord_length * cos(previous.yaw_rad + half_heading_angle);

    //Calculate the new world y coordinate
    new_pose.y_W_m = previous.y_W_m + travel_chord_length * sin(previous.yaw_rad + half_heading_angle);
    
    //Calculate the new yaw
    new_pose.yaw_rad = previous.yaw_rad + yaw_rate_rad_s * dt_s;

    //Return those three values as the updated pose
    return new_pose;
}
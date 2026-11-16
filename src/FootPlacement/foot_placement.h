#ifndef FOOT_PLACEMENT
#define FOOT_PLACEMENT

#include <stdio.h>
#include "../BodyMotion/body_motion.h"
#include "../Gait/gait.h"

/*
Where the bottom of foot is in the G frame
double nominal_foot_x_G_m[4];
double nominal_foot_y_G_m[4];
double ground_z_W_m;
*/
typedef struct {
    double nominal_foot_x_G_m[4]; 
    double nominal_foot_y_G_m[4];
    double ground_z_W_m;
} FootPlacementConfig; //Inicialized in main

/*
    Ground position of the ball bottom in the world (Q):

    double x_W_m;
    double y_W_m;
    double z_W_m;
*/
typedef struct {
    double x_W_m;
    double y_W_m;
    double z_W_m;
} WorldFootprint;

/*
Calculates the next world ground footprint at lift-off, using the predicted next mid-stance pose.

BodyPose2D body_at_liftoff,
double forward_velocity_m_s,
double yaw_rate_rad_s,
const GaitConfig *gait,
double nominal_foot_x_G_m,
double nominal_foot_y_G_m,
double ground_z_W_m
*/
WorldFootprint foot_placement_at_liftoff(
    BodyPose2D body_at_liftoff,
    double forward_velocity_m_s,
    double yaw_rate_rad_s,
    const GaitConfig *gait,
    double nominal_foot_x_G_m, //f_x = h_ix + x_c, measured from the body origin in G [m]
    double nominal_foot_y_G_m, //f_y = h_iy + s_i*Y, measured from the body origin in G [m]
    double ground_z_W_m //World ground height [m]
);


//Calculate Q from the body pose already evaluated at the stance midpoint
WorldFootprint footprint_from_midstance_pose(
    BodyPose2D midstance_pose,
    double nominal_foot_x_G_m,
    double nominal_foot_y_G_m,
    double ground_z_W_m
);

#endif // FOOT_PLACEMENT

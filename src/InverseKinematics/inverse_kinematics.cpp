#include "inverse_kinematics.h"
#include <math.h>

LegJointAngles leg_inverse_kinematics(
    HipFootPosition foot_center_H,
    double upper_link_length_m,
    double lower_link_length_m,
    const double hip_roll_to_pitch_m[3],
    double upper_lateral_offset_m,
    double lower_lateral_offset_m
) {

    LegJointAngles angles;

    // Same symbols as paper equations
    double x_H = foot_center_H.x_H_m;
    double y_H = foot_center_H.y_H_m;
    double z_H = foot_center_H.z_H_m;
    double L1 = upper_link_length_m;
    double L2 = lower_link_length_m;
    double d_x = hip_roll_to_pitch_m[0];
    double d_y = hip_roll_to_pitch_m[1];
    double d_z = hip_roll_to_pitch_m[2];
    double a1 = upper_lateral_offset_m;
    double a2 = lower_lateral_offset_m;

    //1. Roll triangle: d_y_eff = d_y + a1 + a2; unrolled vertical coordinate is -r
    double d_y_eff = d_y + a1 + a2;
    double r_squared = y_H * y_H + z_H * z_H - d_y_eff * d_y_eff;
    double r = sqrt(r_squared);

    //2. Hip roll: alpha = target direction - unrolled direction
    angles.alpha_rad = atan2(y_H, -z_H) - atan2(d_y_eff, r);

    //3. Undo roll: c_M = R_x(-alpha) * c_H = {x_H, d_y_eff, -r}
    //Subtract d to reach the pitch origin; solve its x-z projection
    double x_F = x_H - d_x;
    double z_F = -r - d_z;
    double D = sqrt(x_F * x_F + z_F * z_F);

    //4. Knee: theta_2 = acos((D^2 - L1^2 - L2^2) / (2*L1*L2))
    double cos_theta_2 = (D * D - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    angles.theta_2_rad = acos(cos_theta_2); //Keep the positive-knee branch

    //5. Hip pitch: theta_1 = gamma - b_triangle
    double gamma = atan2(x_F, -z_F);
    double cos_b_triangle = (L1 * L1 + D * D - L2 * L2) / (2 * L1 * D);
    double b_triangle = acos(cos_b_triangle);
    angles.theta_1_rad = gamma - b_triangle;

    return angles; //All three mathematical joint angles in radians
}

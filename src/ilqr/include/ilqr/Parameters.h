#pragma once

#include <iostream>

class Parameters
{
public:
    Parameters();

public:
    // carla paramters

    int num_obstacle;

    // planning parrameters
    int num_of_local_wpts;
    int poly_order;
    int use_pid; // bool
    double desired_speed;
    int use_mpc; // bool
    int mpc_horizon;

    // iLQR parameters
    double timestep;
    int horizon;
    double tolerance;
    int max_iterations;
    int num_states;
    int num_ctrls;

    // cost parameters
    double w_acc;
    double w_yawrate;

    double w_pos;
    double w_vel;

    double w_obstacle;
    double w_uncertainty;

    // cost parameters barrier function q1 q2
    double q1_acc;
    double q2_acc;

    double q1_yawrate;
    double q2_yawrate;

    double q1_front;
    double q2_front;

    double q1_rear;
    double q2_rear;

    double q1_uncertainty;
    double q2_uncertainty;

    // constraint limit
    double acc_max;
    double acc_min;

    double steer_angle_min;
    double steer_angle_max;

    // ego vehicle parameters

    double wheelbase;
    double speed_max;

    double steer_control_max;
    double steer_control_min;

    double throttle_control_max;
    double throttle_control_min;
    // guideline parameters
    double w_guideline;

    // obstacle parameters
    double t_safe;    // time safety
    double s_safe_a;  // safety margin head
    double s_safe_b;  // safety margin side
    double ego_rad;   // radius
    double ego_front; // front distance
    double ego_rear;  // rear distance

    double length;
    double width;

    double safe_length;
    double safe_width;

    //
};
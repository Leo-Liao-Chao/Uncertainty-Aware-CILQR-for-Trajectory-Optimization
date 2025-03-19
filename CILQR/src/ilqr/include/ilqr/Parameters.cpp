#include "Parameters.h"

Parameters::Parameters()
{
        // planning parrameters
        num_of_local_wpts = 20;
        poly_order = 5;
        desired_speed = 5.0;

        // iLQR parameters
        timestep = 0.1;
        horizon = 40;
        tolerance = 1e-4;
        max_iterations = 20;
        num_states = 4;
        num_ctrls = 2;

        // cost parameters
        w_acc = 1.0;
        w_yawrate = 4.0;

        w_pos = 0.65;//1.0 0.75 0.65
        w_vel = 3.0;//3.0

        w_obstacle = 1.0;
        w_uncertainty = 1.0;

        // cost parameters barrier function q1 q2
        q1_acc = 1.00;
        q2_acc = 1.00;

        q1_yawrate = 1.0;
        q2_yawrate = 1.0;

        q1_front = 2.75;
        q2_front = 2.75;

        q1_rear = 2.5;
        q2_rear = 2.5;

        q1_uncertainty = 2.5;
        q2_uncertainty = 2.5;

        // constralimit
        acc_max = 2.0;
        acc_min = -5.5;

        steer_angle_min = -0.75; // m
        steer_angle_max = 0.75;  // m

        // ego vehicle parameters

        wheelbase = 2.94;
        speed_max = 30.0;

        steer_control_max = 1.0;  // rad
        steer_control_min = -1.0; // rad

        throttle_control_max = 1.0;  // rad
        throttle_control_min = -1.0; // rad

        // obstacle parameters
        t_safe = 0.1;
        s_safe_a = 0; // LENGTH 5.8
        s_safe_b = 0; // WIDTH 2.1
        ego_rad = 1.35;
        ego_front = 1.47 + 0.925;
        ego_rear = 1.47 + 0.925;

        length = 4.79;
        width = 2.16;

        safe_length = 0.0;
        safe_width = 0.0;
}
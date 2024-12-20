#include "Obstacle.h"

Obstacle::Obstacle(Parameters p)
{
    this->p = p;
}

Obstacle::Obstacle(Parameters p, Eigen::MatrixXd dimension, Eigen::MatrixXd relative_pos_array)
{
    this->p = p;
    this->dimension = dimension;
    this->relative_pos_array = relative_pos_array;
}
/*
input:q1,q2,c,c_dot:2*1
output:
x:q1*e^(q2*c)
vx:[x,x].T 2*1
mx:[x,x;x,x;] 2*2
*/
struct_x_vx_mx_vmx Obstacle::barrier_function(double q1, double q2, double c, Eigen::Vector4d c_dot)
{
    struct_x_vx_mx_vmx result;
    result.x = q1 * std::exp(q2 * c);
    result.vx = q2 * q1 * std::exp(q2 * c) * c_dot;

    Eigen::MatrixXd c_dot_m = c_dot;
    Eigen::MatrixXd c_dot_m_t = c_dot.transpose();

    result.mx = q2 * q2 * q1 * std::exp(q2 * c) * c_dot_m * c_dot_m_t;
    return result;
}

/*

vx:4*1
mx:4*4
*/
struct_x_vx_mx_vmx Obstacle::get_obstalce_cost(int i, Eigen::VectorXd ego_state)
{
    Eigen::VectorXd obstalce_pos = this->relative_pos_array.col(i);
    double a = this->dimension(0, i) / 2.0 + std::abs(obstalce_pos(2) * std::cos(obstalce_pos(3))) * this->p.t_safe + this->p.s_safe_a + this->p.ego_rad;
    double b = this->dimension(1, i) / 2.0 + std::abs(obstalce_pos(2) * std::sin(obstalce_pos(3))) * this->p.t_safe + this->p.s_safe_b + this->p.ego_rad + 1;
    Eigen::MatrixXd P1 = Eigen::MatrixXd::Identity(4, 4);
    P1(0, 0) = 1.0 / a / a;
    P1(1, 1) = 1.0 / b / b;
    P1(2, 2) = 0.0;
    P1(3, 3) = 0.0;

    double obs_theta = obstalce_pos(3);
    double vehicle_theta = ego_state(3);

    Eigen::MatrixXd tf_martix = Eigen::MatrixXd::Zero(4, 4);
    tf_martix(0, 0) = std::cos(obs_theta);
    tf_martix(0, 1) = std::sin(obs_theta);
    tf_martix(1, 0) = -std::sin(obs_theta);
    tf_martix(1, 1) = std::cos(obs_theta);
    Eigen::MatrixXd tf_martix_reverse = Eigen::MatrixXd::Zero(4, 4);
    tf_martix_reverse(0, 0) = std::cos(-obs_theta); // Test
    tf_martix_reverse(0, 1) = std::sin(-obs_theta); // Test
    tf_martix_reverse(1, 0) = -std::sin(-obs_theta); // Test
    tf_martix_reverse(1, 1) = std::cos(-obs_theta); // Test

    /* global diff*/
    Eigen::VectorXd ego_front = ego_state;

    ego_front(0) = ego_state(0) + std::cos(vehicle_theta) * this->p.ego_front;
    ego_front(1) = ego_state(1) + std::sin(vehicle_theta) * this->p.ego_front;
    Eigen::VectorXd diff = (tf_martix *(ego_front - obstalce_pos)).eval();
    /*relative diff*/

    // Eigen::VectorXd ego_front_diff = obstalce_pos;

    // ego_front_diff(0) = ego_front_diff(0) + std::cos(vehicle_theta) * this->p.ego_front;
    // ego_front_diff(1) = ego_front_diff(1) + std::sin(vehicle_theta) * this->p.ego_front;
    // Eigen::VectorXd diff = ((tf_martix*ego_front_diff)).eval();

    Eigen::VectorXd temp = diff.transpose() * P1 * diff;
    double c = 1 - temp(0);
    // std::cout<<"c: %.2f"<<c<<std::endl;

    Eigen::VectorXd c_dot = (-2) * tf_martix_reverse * P1 * diff;
    struct_x_vx_mx_vmx obs_front = this->barrier_function(this->p.q1_front, this->p.q2_front, c, c_dot);
    
    /* global diff*/
    Eigen::VectorXd ego_rear = ego_state;

    ego_rear(0) = ego_state(0) - std::cos(vehicle_theta) * this->p.ego_rear;
    ego_rear(1) = ego_state(1) - std::sin(vehicle_theta) * this->p.ego_rear;
    diff = ( tf_martix * (ego_rear - obstalce_pos)).eval();

    /* relative diff*/
    // Eigen::VectorXd ego_rear_diff = obstalce_pos;

    // ego_rear_diff(0) = ego_rear_diff(0) - std::cos(vehicle_theta) * this->p.ego_rear;
    // ego_rear_diff(1) = ego_rear_diff(1) - std::sin(vehicle_theta) * this->p.ego_rear;
    // diff = (tf_martix * ego_rear_diff).eval();
    
    temp = diff.transpose() * P1 * diff;
    c = 1 - temp(0);
    c_dot = (-2) * tf_martix_reverse * P1 * diff;

    struct_x_vx_mx_vmx obs_rear = this->barrier_function(this->p.q1_rear, this->p.q2_rear, c, c_dot);

    Eigen::VectorXd obs_d = obs_front.vx + obs_rear.vx;
    Eigen::MatrixXd obs_dd = obs_front.mx + obs_rear.mx;

    struct_x_vx_mx_vmx result;
    result.vx = obs_d;
    result.mx = obs_dd;
    return result;
}


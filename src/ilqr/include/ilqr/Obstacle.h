#pragma once

#define EIGEN_USE_MKL_ALL
#include "mkl.h"

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "Parameters.h"
#include "struct_x_vx_mx_vmx.h"

class Obstacle
{
public:
    Obstacle(Parameters p);
    Obstacle(Parameters p, Eigen::MatrixXd dimension, Eigen::MatrixXd relative_pos_array);

    struct_x_vx_mx_vmx barrier_function(double q1, double q2, double c, Eigen::Vector4d c_dot);
    struct_x_vx_mx_vmx get_obstalce_cost(int i, Eigen::VectorXd ego_state);

private:
    Parameters p;
    Eigen::MatrixXd dimension;          // obstalce size
    Eigen::MatrixXd relative_pos_array; // 4*n
};
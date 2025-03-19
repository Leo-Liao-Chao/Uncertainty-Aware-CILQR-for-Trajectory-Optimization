#pragma once

#define EIGEN_USE_MKL_ALL
#include "mkl.h"

#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>
#include <vector>

/*
double x; vectorxd vx; matrixxd mx; vector<matrixxd> vmx;
*/
struct struct_x_vx_mx_vmx
{
    double x;
    Eigen::VectorXd vx;
    Eigen::MatrixXd mx;
    Eigen::Tensor<double, 3> tx;
    std::vector<Eigen::MatrixXd> vmx;
};
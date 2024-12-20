#pragma once

#define EIGEN_USE_MKL_ALL
#include "mkl.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

#include "Parameters.h"

class Model
{
public:
    Model(const Parameters &params);

    Eigen::Vector4d forward_simulate(const Eigen::Vector4d &state, Eigen::Vector2d control);
    // std::vector<Eigen::MatrixXd> vector_get_A_matrix(const Eigen::VectorXd &velocity_vals, const Eigen::VectorXd &theta, const Eigen::VectorXd &acceleration_vals);
    // std::vector<Eigen::MatrixXd> vector_get_B_matrix(const Eigen::VectorXd &theta);
    Eigen::Tensor<double,3> get_A_matrix(const Eigen::VectorXd &velocity_vals, const Eigen::VectorXd &theta, const Eigen::VectorXd &acceleration_vals);
    Eigen::Tensor<double,3> get_B_matrix(const Eigen::VectorXd &theta);


private:
    Parameters params;
};

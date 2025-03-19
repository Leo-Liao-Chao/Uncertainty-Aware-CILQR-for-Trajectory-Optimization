#pragma once

#define EIGEN_USE_MKL_ALL
#include "mkl.h"

#include <iostream>
#include <vector>
#include <limits>
#include <unsupported/Eigen/CXX11/Tensor>

#include "Parameters.h"
#include "LocalPlanner.h"
#include "Constraints.h"
#include "Model.h"

class iLQR
{
public:
    iLQR(const Parameters &params);
    void set_Obstacle(const std::vector<Obstacle> &obstacles);
    void clear_Obstacle();
    void set_uncertainty_map(const Uncertainty &uncertainty);
    void clear_uncertainty_map();
    
    void set_global_plan(const Eigen::MatrixXd &global_plan);
    Eigen::MatrixXd get_nominal_trajectory(const Eigen::VectorXd &x_0, const Eigen::MatrixXd &U);

    void forward_pass(const Eigen::MatrixXd &X, const Eigen::MatrixXd &U, const Eigen::MatrixXd &k, const Eigen::Tensor<double, 3> &K);
    bool backward_pass(const Eigen::MatrixXd &X, const Eigen::MatrixXd &U, const Eigen::VectorXd &ploy_coeffs, const Eigen::VectorXd &x_local_plan, const double &lamb);
    void get_optimal_control_seq(const Eigen::VectorXd &x_0, Eigen::MatrixXd &U, const Eigen::VectorXd &poly_coeffs, const Eigen::VectorXd &x_local_plan);
    void run_step(const Eigen::VectorXd &ego_state);

private:
    Eigen::MatrixXd control_seq;

    Eigen::MatrixXd global_plan;

    double lamb_factor;
    double lamb_max;

    // forward
    Eigen::MatrixXd X_new;
    Eigen::MatrixXd U_new;

    // backward
    Eigen::MatrixXd k;
    Eigen::Tensor<double, 3> K;

public:
    Parameters params;
    LocalPlanner localplanner;
    Constraints constraints;
    Model model;

    // result
    Eigen::MatrixXd X_result;
    Eigen::MatrixXd U_result;
    Eigen::MatrixXd ref_traj_result;
};
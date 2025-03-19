#pragma once

#define EIGEN_USE_MKL_ALL
#include "mkl.h"

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "Parameters.h"

class LocalPlanner
{
public:
    LocalPlanner(const Parameters &param);

    void set_ego_state(const Eigen::Vector4d &ego_state);
    void set_global_planner(const Eigen::MatrixXd &global_plan_points);

    int closest_point_index(const Eigen::Vector4d &point);
    Eigen::MatrixXd get_local_wpts();
    Eigen::MatrixXd get_local_plan();
    Eigen::VectorXd get_local_plan_coeffs();
    Eigen::VectorXd polyfit(const Eigen::VectorXd &x, const Eigen::VectorXd &y, int &degree);

private:
    Eigen::Vector4d ego_state;
    Eigen::MatrixXd global_plan_points;
    Parameters param;
};

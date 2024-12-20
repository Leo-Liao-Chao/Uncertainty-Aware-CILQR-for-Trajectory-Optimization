#pragma once

#define EIGEN_USE_MKL_ALL
#include "mkl.h"

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>
#include <future>
#include <queue>
#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "map_engine/map_param.h"
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "Parameters.h"
#include "struct_x_vx_mx_vmx.h"
#include "UncertaintyMap.h"


// 定义二维高斯分布的参数结构体
struct GaussianParams {
    double mu_x;
    double mu_y;
    double sigma_x;
    double sigma_y;
    double rho;
    double sigma_x_pow2;
    double sigma_y_pow2;
    double rho_pow2;
    double sigma_x_multiple_sigma_y;

};


class Uncertainty
{
public:
    Uncertainty();
    Uncertainty(const Parameters &param, const nav_msgs::OccupancyGrid &map, const double x_center, const double y_center, double sigma_X, double sigma_Y, double sigma_Theta, double sigma_Accelerate, double sigma_AngleVelocity);
    Uncertainty(const Parameters &param, const nav_msgs::OccupancyGrid &map, const grid_map_msgs::GridMap &gridmap, const double x_center, const double y_center, double sigma_X, double sigma_Y, double sigma_Theta, double sigma_Accelerate, double sigma_AngleVelocity,ros::NodeHandle nh);
    
    void InitialUncertaintyMap();

    struct_x_vx_mx_vmx barrier_function(double q1, double q2, double c, Eigen::Vector4d c_dot);
    Eigen::Vector2d GlobalIndexInVehicleMap(Eigen::VectorXd ego_state);
    bool GlobalInVehicleMap(Eigen::VectorXd ego_state);

    Eigen::Matrix2f getConfidenceMatrix();
    std::vector<double> getConfidenceEllipse(Eigen::Matrix2f cov, double chisquare_val);

    
    struct_x_vx_mx_vmx get_uncertainty_cost(Eigen::VectorXd ego_state);
    void ElipseVisualization(std::vector<std::pair<double,double>> &Elipse);

    double gaussian_fx(double x, double y, const GaussianParams& params);
    void gaussian_fx_derivative(double x, double y, const GaussianParams& params,double &fx,Eigen::Vector2d &fx_d);
    void createRotatedRectangle(grid_map::Polygon& polygon, const grid_map::Position& center, double length, double width, double angle);

private:
    Parameters param;
    ros::NodeHandle nh;
    ros::Publisher elipse_pub;

    Eigen::Vector2d PX;
    Eigen::Vector2d PY;

    std::vector<double> elipseParam_const;

    Eigen::Matrix2d uncertainty_cost;

    double sigma_X;
    double sigma_Y;
    double sigma_Theta;

    double sigma_Accelerate;
    double sigma_AngleVelocity;

    // Occupancy map
    nav_msgs::OccupancyGrid map;
    grid_map::GridMap gridmap_uncertaintymap;

    // Index map
    double x_center;
    double y_center;
    UncertaintyMap vehiclemap;

    // Vehicle Map
    double vehicle_length_half;
    double vehicle_width_half;
};

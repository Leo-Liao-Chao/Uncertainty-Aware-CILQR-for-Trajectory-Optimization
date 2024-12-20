#pragma once
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf/transform_datatypes.h"
#include "tf2_ros/buffer.h"

#include <dynamic_reconfigure/server.h>
#include <map_engine/map_engine_Config.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <time.h>

#include <thread>

#include "DBSCAN.h"

#include <thrust/for_each.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/zip_iterator.h>

#include <thrust/detail/config.h>

#if THRUST_CPP_DIALECT >= 2011 && !defined(THRUST_LEGACY_GCC)
#include <thrust/zip_function.h>
#endif // >= C++11

#include <typeinfo>
#include <string>
#include <cxxabi.h>
#include <math.h>

using namespace grid_map;
using namespace std;
using namespace cv;

struct uncertainty_error_functor
{
    
    double sin_Vtheta_og, cos_Vtheta_og, sigma_x, sigma_y, sigma_theta;
    uncertainty_error_functor(double _sin_Vtheta_og, double _cos_Vtheta_og, double _sigma_x, double _sigma_y, double _sigma_theta) :
                              sin_Vtheta_og(_sin_Vtheta_og), cos_Vtheta_og(_cos_Vtheta_og), sigma_x(_sigma_x), sigma_y(_sigma_y), sigma_theta(_sigma_theta) {}
    __host__ __device__
    void operator()(const double& Cx, const double& Cy, double& sigma_x_i, double& sigma_y_i, double& rho)
    {
        double u = (-sin_Vtheta_og * Cx - cos_Vtheta_og * Cy) * (-sin_Vtheta_og * Cx - cos_Vtheta_og * Cy);
        double v = (cos_Vtheta_og * Cx - sin_Vtheta_og * Cy) * (cos_Vtheta_og * Cx - sin_Vtheta_og * Cy);
        double t = sin_Vtheta_og * cos_Vtheta_og * (Cx * Cx - Cy * Cy)
            + Cx * Cy * (sin_Vtheta_og * sin_Vtheta_og - cos_Vtheta_og * cos_Vtheta_og);
        
        sigma_x_i = sqrt(sigma_x * sigma_x + sigma_theta * sigma_theta * u);
        sigma_y_i = sqrt(sigma_y * sigma_y + sigma_theta * sigma_theta * v);
        rho = sigma_theta * sigma_theta * t / (sigma_x_i * sigma_y_i);
    }
};

struct abc_functor
{
    __host__ __device__
    void operator()(const double& sigma_x_i, const double& sigma_y_i, const double& rho, double& a, double& b, double& c)
    {
        a = sigma_x_i * sigma_x_i;
        b = rho * sigma_x_i * sigma_y_i;
        c = sigma_y_i * sigma_y_i;
    }
};

struct ellipse_params_functor
{
    __host__ __device__
    void operator()(const double& a, const double& b, const double& c, int& major_index, int& minor_index, Eigen::Matrix2f& D, Eigen::Matrix2f& V, double& half_major_axis, double& half_minor_axis, double& angle)
    {
        double chisquare_val=2.4477;
        // Calculate the angle between the largest eigenvector and the x-axis
        angle = atan2(V(major_index,1), V(major_index,0));
        if (angle < 0)
        {
            angle += 6.28318530718;
        }

        // Calculate the size of the minor and major axes
        half_major_axis = chisquare_val * sqrt(D(major_index,major_index));
        half_minor_axis = chisquare_val * sqrt(D(minor_index,minor_index));
    }
};


// 2-d nomal distribution
__host__ __device__
inline double nomal2(double x, double y, double mu1, double mu2, double sigma1, double sigma2, double rho)
{
    return 1.0/(sqrt(1-rho*rho)*(2*M_PI*sigma1*sigma2)) * exp((-1/(2*(1-rho*rho)))*((x-mu1)*(x-mu1)/(sigma1*sigma1)-2*rho*(x-mu1)*(y-mu2)/(sigma1*sigma2)+(y-mu2)*(y-mu2)/(sigma2*sigma2)));
}

vector<pair<int, double>> thrust_propagateUncertainty(grid_map::GridMap vehicle_map_, int index, double sin_Vtheta_og, double cos_Vtheta_og, double sigma_x, double sigma_y, double sigma_theta);


// struct u_functor
// {
//     __host__ __device__
//     void operator()(const double& Cx, const double& Cy, const double& sin_Vtheta_og, const double& cos_Vtheta_og, double& u)
//     {
//         u = (-sin_Vtheta_og * Cx - cos_Vtheta_og * Cy) * (-sin_Vtheta_og * Cx - cos_Vtheta_og * Cy);
//     }
// };

// struct v_functor
// {
//     __host__ __device__
//     void operator()(const double& Cx, const double& Cy, const double& sin_Vtheta_og, const double& cos_Vtheta_og, double& v)
//     {
//         v = (cos_Vtheta_og * Cx - sin_Vtheta_og * Cy) * (cos_Vtheta_og * Cx - sin_Vtheta_og * Cy);
//     }
// };

// struct t_functor
// {
//     __host__ __device__
//     void operator()(const double& Cx, const double& Cy, const double& sin_Vtheta_og, const double& cos_Vtheta_og, double& t)
//     {
//         t = sin_Vtheta_og * cos_Vtheta_og * (Cx * Cx - Cy * Cy)
//             + Cx * Cy * (sin_Vtheta_og * sin_Vtheta_og - cos_Vtheta_og * cos_Vtheta_og);
//     }
// };

// struct sigma_x_i_functor
// {
//     __host__ __device__
//     void operator()(const double& sigma_x, const double& sigma_theta, const double& u, double& sigma_x_i)
//     {
//         sigma_x_i = sqrt(sigma_x * sigma_x + sigma_theta * sigma_theta * u);
//     }
// };

// struct sigma_y_i_functor
// {
//     __host__ __device__
//     void operator()(const double& sigma_y, const double& sigma_theta, const double& v, double& sigma_y_i)
//     {
//         sigma_y_i = sqrt(sigma_y * sigma_y + sigma_theta * sigma_theta * v);
//     }
// };

// struct rho_functor
// {
//     __host__ __device__
//     void operator()(const double& sigma_x_i, const double& sigma_y_i, const double& sigma_theta, const double& t, double& rho)
//     {
//         rho = sigma_theta * sigma_theta * t / (sigma_x_i * sigma_y_i);
//     }
// };

// struct a_functor
// {
//     __host__ __device__
//     void operator()(const double& sigma_x_i, double& a)
//     {
//         a = sigma_x_i * sigma_x_i;
//     }
// };

// struct b_functor
// {
//     __host__ __device__
//     void operator()(const double& sigma_x_i, const double& sigma_y_i, const double& rho, double& b)
//     {
//         b = rho * sigma_x_i * sigma_y_i;
//     }
// };

// struct c_functor
// {
//     __host__ __device__
//     void operator()(const double& sigma_y_i, double& c)
//     {
//         c = sigma_y_i * sigma_y_i;
//     }
// };


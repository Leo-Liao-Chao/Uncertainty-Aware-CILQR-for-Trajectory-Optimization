#pragma once

#define EIGEN_USE_MKL_ALL
#include "mkl.h"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <time.h>
#include <iostream>
#include <chrono>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "carla_msgs/CarlaEgoVehicleControl.h"
#include "ackermann_msgs/AckermannDrive.h"

#include "map_engine/map_param.h"
#include <dynamic_reconfigure/client.h>
#include <map_engine/map_engine_Config.h>

#include "iLQR.h"
#include "Parameters.h"
#include "vehiclepub/VehicleInfo.h"
#include "vehiclepub/VehicleInfoArray.h"
#include "vehiclepub/Experiment.h"

#include "Uncertainty.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class ilqr_uncertainty_node
{
private:
    Parameters params;
    iLQR ilqrplanner;
    MatrixXd global_path;
    VectorXd current_ego_vehicle_state;
    VectorXd current_ego_vehicle_state_noise;

    ros::NodeHandle nh;
    ros::Subscriber lane_info_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber map_sub;
    ros::Subscriber static_obstacle_sub;
    ros::Subscriber map_param_sub;
    ros::Subscriber grid_map_sub;
    ros::Publisher ilqr_path_pub;
    ros::Publisher vehicle_cmd_pub;
    ros::Publisher experiment_data_pub;

    // dynamic reconfigure
    // dynamic_reconfigure::Client<map_engine::map_engine_Config> client;
    // map_engine::map_engine_Config config;
    // dynamic_reconfigure::Server<map_engine::map_engine_Config> server;
    // dynamic_reconfigure::Server<map_engine::map_engine_Config>::CallbackType f;
    // position transform
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // uncertainty params

    double SIGMA_X;
    double SIGMA_Y;
    double SIGMA_THETA;

    // vechicle map params

    double X_LENGTH;
    double Y_LENGTH;
    double X_POSITION;
    double Y_POSITION;
    double RESOLUTION;

    // uncertainty map = occupancy map
    nav_msgs::OccupancyGrid map_msg;
    double x_center;
    double y_center;

    // uncertainty map ->grid map
    grid_map_msgs::GridMap grid_map_msg;

    void laneInfoCallback(const nav_msgs::Path::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
    void PathVisualization(MatrixXd &path);
    void publishVehicleCmd(const double &speed, const double &angle, const double &accel, const double &angle_velocity);
    void publishExperimentData(ros::Time start_time, 
                             const VectorXd &start_pos, 
                             const double & planning_time, 
                             const MatrixXd &X, 
                             const MatrixXd &U );
    void staticobstacleCallback(const vehiclepub::VehicleInfoArray::ConstPtr &message);
    void mapCallback(const nav_msgs::OccupancyGrid &message);
    void mapParamCallback(const map_engine::map_param &message);
    void gridmapCallback(const grid_map_msgs::GridMap &message);

    MatrixXd pathInGlobal2Vechicle(const MatrixXd &global_path, const VectorXd &ego_state);
    MatrixXd pathInVechicle2Global(const MatrixXd &vehicle_path, const VectorXd &ego_state);



public:
    ilqr_uncertainty_node();
};

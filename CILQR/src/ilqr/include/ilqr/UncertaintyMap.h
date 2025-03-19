#pragma once

#define EIGEN_USE_MKL_ALL
#include "mkl.h"

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>

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

#include "Parameters.h"
#include "struct_x_vx_mx_vmx.h"

class UncertaintyMap
{
public:
    int data[500][500];
    double heading; // vehicle map heading in global frame
    double Vx_og;   // vehicle map x in global frame
    double Vy_og;   // vehicle map y in global frame
    int width;
    int height;
    double resolution;
    double x_center; // vehicle map center's x coordination
    double y_center; // vehicle map center's y coordination

    UncertaintyMap()
    {
        memset(data, -1, sizeof(data));
        heading = 0.0;
        Vx_og = 0.0;
        Vy_og = 0.0;
        width = 0;
        height = 0;
        resolution = 0.2;
        x_center = 0.0;
        y_center = 0.0;
    }
};
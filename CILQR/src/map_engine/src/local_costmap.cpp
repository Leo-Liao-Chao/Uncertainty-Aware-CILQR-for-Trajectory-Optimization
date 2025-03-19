#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf/transform_datatypes.h"
#include "map_engine/map_param.h"
#include "carla_msgs/CarlaEgoVehicleInfo.h"
#include "visualization_msgs/MarkerArray.h"

#include <dynamic_reconfigure/server.h>
#include <map_engine/map_engine_Config.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <time.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video/tracking.hpp>

#include <thread>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>

#include "DBSCAN.h"
#include "ARBIT.cuh"

#include "vehiclepub/VehicleInfo.h"
#include "vehiclepub/VehicleInfoArray.h"

#define THREADS_NUM 1
#define LOOK_AHEAD_DISTENCE 40

using namespace grid_map;
using namespace std;
using namespace cv;

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

/* List of dynamic parameters */

// Dynamic parameter server callback function
void dynamicParamCallback(map_engine::map_engine_Config &config, uint32_t level)
{
    SIGMA_X = config.sigma_x;
    SIGMA_Y = config.sigma_y;
    SIGMA_THETA = config.sigma_theta;
    X_LENGTH = config.x_length;
    Y_LENGTH = config.y_length;
    X_POSITION = config.x_position;
    Y_POSITION = config.y_position;
    RESOLUTION = config.resolution;
}

// 2-d nomal distribution
// inline double nomal2(double x, double y, double mu1, double mu2, double sigma1, double sigma2, double rho) {
//     return 1.0 / (sqrt(1 - rho * rho) * (2 * M_PI * sigma1 * sigma2)) * exp((-1 / (2 * (1 - rho * rho))) * ((x - mu1) * (x - mu1) / (sigma1 * sigma1) - 2 * rho * (x - mu1) * (y - mu2) / (sigma1 * sigma2) + (y - mu2) * (y - mu2) / (sigma2 * sigma2)));
// }

class LocalCostmap
{
public:
    LocalCostmap(ros::NodeHandle &nodeHandle)
        : nodeHandle_(nodeHandle), tf_listener(tf_buffer)
    {
        // subscribe
        sub_global_map_ = nodeHandle_.subscribe("/map", 1, &LocalCostmap::globalMapCallback, this);
        sub_odom_ = nodeHandle_.subscribe("/carla/ego_vehicle/odometry", 1, &LocalCostmap::odomCallback, this);
        sub_lidar_ = nodeHandle_.subscribe("/LiDARGridMap", 1, &LocalCostmap::lidarMapCallback, this);
        sub_bbox_ = nodeHandle_.subscribe("/bnd_box", 1, &LocalCostmap::bboxCallback, this);
        sub_bbox_f_ = nodeHandle_.subscribe("/bnd_box_f", 1, &LocalCostmap::bboxFCallback, this);
        sub_bonding_box_ = nodeHandle_.subscribe("/obstacle", 1, &LocalCostmap::bondingBoxCallback, this);
        sub_waypoints_ = nodeHandle_.subscribe("/carla/ego_vehicle/waypoints", 1, &LocalCostmap::waypointsCallback, this);
        sub_vehicle_info_ = nodeHandle_.subscribe("/carla/ego_vehicle/vehicle_info", 1, &LocalCostmap::vehicleInfoCallback, this);
        sub_static_obstacle_ = nodeHandle_.subscribe("/static_obstacle/vehicle_info", 1, &LocalCostmap::staticobstacleCallback, this);

        // advertiser
        pub_local_costmap_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("vehicle_map", 1, true);
        pub_for_planner_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("for_planner_map", 1, true);
        pub_global_costmap_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("global_map", 1, true);
        pub_map_param_ = nodeHandle_.advertise<map_engine::map_param>("map_param", 1, true);
        pub_grid_num_ = nodeHandle_.advertise<std_msgs::Int32>("grid_num", 1, true);
        pub_compute_time_ = nodeHandle_.advertise<std_msgs::Int32>("compute_time", 1, true);

        pub_for_planner_grid_map_uncertainty_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("for_planner_map_grid_map", 1, true);

        // visualization for Rviz
        visual_vehicle_map_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("test_map", 1, true);
        visual_semantic_lidar_map_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("semantic_lidar_map", 1, true);

        // global_map
        global_map_.add("global_map");
        global_map_.setFrameId("map");
        global_map_.setGeometry(Length(301.2, 301.2), 0.2, Position(93.14, -205.96));
        ROS_INFO("Created global map with size %f x %f m (%i x %i cells).",
                 global_map_.getLength().x(), global_map_.getLength().y(),
                 global_map_.getSize()(0), global_map_.getSize()(1));

        // vehicle_map
        vehicle_map_.add("vehicle_map");        // initial map layer
        vehicle_map_.add("uncertainty_map");    // uncertainty map layer
        vehicle_map_.add("lidar_grid_map");     // original lidar data
        vehicle_map_.add("semantic_lidar_map"); // 1st edition bonding box layer
        vehicle_map_.add("bounding_box_map");   // 2ed edition bonding box layer
        vehicle_map_.add("ellipse_map");        // for visual uncertainty ellipse
        vehicle_map_.setFrameId("ego_vehicle");
        vehicle_map_.setGeometry(Length(30, 20), 0.2, Position(15, 0));

        // dynamic server
        f = boost::bind(&dynamicParamCallback, _1, _2);
        server.setCallback(f);

        // kalman filter setting
        stateNum = 6;
        measureNum = 4;
        KF = KalmanFilter(stateNum, measureNum, 0);
        // Mat processNoise(stateNum, 1, CV_32F);
        measurement = Mat::zeros(measureNum, 1, CV_32F);

        KF.transitionMatrix = (Mat_<double>(stateNum, stateNum) // 状态转移矩阵A ，控制矩阵B默认为零
                                   << 1,
                               0, 0, 0, 1, 0,
                               0, 1, 0, 0, 0, 1,
                               0, 0, 1, 0, 0, 0,
                               0, 0, 0, 1, 0, 0,
                               0, 0, 0, 0, 1, 0,
                               0, 0, 0, 0, 0, 1);

        setIdentity(KF.measurementMatrix);                      // 测量矩阵H
        setIdentity(KF.processNoiseCov, Scalar::all(1e-5));     // 系统噪声方差矩阵Q，高斯白噪声，单位阵
        setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1)); // 测量噪声方差矩阵R，高斯白噪声，单位阵
        setIdentity(KF.errorCovPost, Scalar::all(1));           // 后验错误估计协方差矩阵P，初始化为单位阵
        randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));  // 初始化状态为随机值
        memset(formerX, 0, sizeof(int) * 4);
    }

    ~LocalCostmap()
    {
    }

    // get global map from topic
    void globalMapCallback(const nav_msgs::OccupancyGrid &message)
    {
        grid_map::GridMapRosConverter::fromOccupancyGrid(message, "global_map", this->global_map_);
    }

    void odomCallback(const nav_msgs::Odometry &message)
    {
        /*
        1. First get the car`s current pose
        */
        double Vx_og, Vy_og, Vtheta_og; // vehicle position and heading in the frame Og
        double Cx, Cy;                  // coodinates of the cell i of grid in the frame Ov
        double x_og, y_og;              // position of each cell i in the global frame

        sigma_x = SIGMA_X;         // x-axis uncertainty (meter)
        sigma_y = SIGMA_Y;         // y-axis uncertainty (meter)
        sigma_theta = SIGMA_THETA; // heading uncertainty (rad)

        // transform Quaternion to roll, pitch, yaw
        tf::Quaternion quat;
        tf::quaternionMsgToTF(message.pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // get vehicle current pose
        Vx_og = message.pose.pose.position.x;
        Vy_og = message.pose.pose.position.y;
        Vtheta_og = yaw;
        if (Vtheta_og < 0)
        {
            Vtheta_og += 6.28318530718;
        }

        // Temporary variables for improving efficiency
        double sin_Vtheta_og = sin(Vtheta_og);
        double cos_Vtheta_og = cos(Vtheta_og);

        /*
        2. Calculate current map`s scale
        */
        searchTargetIndex(Vx_og, Vy_og);
        // ROS_INFO("Nearest index %d ", old_nearest_point_index);

        // get the scale of the vehicle map
        getVehicleMapScale();
        // This step will clear the map data, so add data later
        vehicle_map_.setGeometry(Length(X_LENGTH, Y_LENGTH), RESOLUTION, Position(X_POSITION-5, Y_POSITION));
        // ROS_INFO("Created vehicle map with size %f x %f m (%i x %i cells).",
        //          vehicle_map_.getLength().x(), vehicle_map_.getLength().y(),
        //          vehicle_map_.getSize()(0), vehicle_map_.getSize()(1));

        /*
        3. Add data to vehicle map, data from a.perception [bbox], b.prior-map [global map]
        */
        // grid_map::GridMapRosConverter::fromOccupancyGrid(lidarRaw, "lidar_grid_map", vehicle_map_);
        // // only need obstacles points
        // for (grid_map::GridMapIterator iterator(vehicle_map_); !iterator.isPastEnd(); ++iterator) {
        //     if (vehicle_map_.at("lidar_grid_map", *iterator) < 90) {
        //         vehicle_map_.at("lidar_grid_map", *iterator) = -1;
        //     }
        // }

        // bondingBoxHandle1();
        // bondingBoxHandle();

        bondingBoxHandle(this->msg_vehicles, this->vehicle_map_,
                         Vx_og, Vy_og, Vtheta_og);

        clock_t start,
            finish;
        start = clock();
        vector<double> times;
        ros::Time start_time = ros::Time::now();

        // get the initial probability of vehicle frame from priori-map and LiDAR
        for (GridMapIterator it(vehicle_map_); !it.isPastEnd(); ++it)
        {
            Position position;
            vehicle_map_.getPosition(*it, position);
            Cx = position.x();
            Cy = position.y();

            x_og = (Cx * cos_Vtheta_og - Cy * sin_Vtheta_og) + Vx_og;
            y_og = (Cx * sin_Vtheta_og + Cy * cos_Vtheta_og) + Vy_og;

            // ROS_INFO("%f, %f, %f, %f, %f, %f", Cx, Cy, Cx * cos_Vtheta_og - Cy * sin_Vtheta_og, (Cx * sin_Vtheta_og + Cy * cos_Vtheta_og), x_og, y_og);
            vehicle_map_.at("vehicle_map", *it) = global_map_.atPosition("global_map", Position(x_og, y_og));
            // if (vehicle_map_.at("lidar_grid_map", *it) > 90) {
            //     vehicle_map_.at("vehicle_map", *it) = vehicle_map_.at("lidar_grid_map", *it);
            // }
            // if (vehicle_map_.at("semantic_lidar_map", *it) > 90) {
            //     vehicle_map_.at("vehicle_map", *it) = vehicle_map_.at("semantic_lidar_map", *it);
            // }
            if (vehicle_map_.at("bounding_box_map", *it) > 90)
            {
                vehicle_map_.at("vehicle_map", *it) = vehicle_map_.at("bounding_box_map", *it);
            }
        }
        // visualMap("vehicle_map");

        finish = clock();
        times.push_back((double)(finish - start) / CLOCKS_PER_SEC); // times[0]
        times.push_back(0.0);                                       // times[1]
        times.push_back(0.0);                                       // times[2]
        times.push_back(0.0);                                       // times[3]

        // ROS_INFO("Initiate map time : %f s", ros::Time::now().toSec() - start_time.toSec());

        start = clock();
        // propagate uncertainty

        std::thread threads[THREADS_NUM];
        // spawn threads:
        for (int i = 0; i < THREADS_NUM; ++i)
        {
            threads[i] = std::thread(&LocalCostmap::propagateUncertainty, this, i * (((X_LENGTH * Y_LENGTH) / (RESOLUTION * RESOLUTION)) / THREADS_NUM), sin_Vtheta_og, cos_Vtheta_og, Cx, Cy);
        }
        for (auto &th : threads)
            th.join();

        ros::Time end_time = ros::Time::now();
        // ROS_INFO("cost time : %f s", end_time.toSec() - start_time.toSec());
        finish = clock();
        times[3] = (double)(finish - start) / CLOCKS_PER_SEC;
        // ROS_INFO("times[0] %f times[1] %f times[2] %f times[3] %f", times[0], times[1], times[2], times[3] / 15000);
        // ROS_INFO("times[0] %f times[3] %f", times[0], times[3]);

        // Publish vehicle map.

        vehicle_map_.setTimestamp(ros::Time::now().toNSec());
        nav_msgs::OccupancyGrid vehicle_map_message, for_planner_message;
        GridMapRosConverter::toOccupancyGrid(vehicle_map_, "uncertainty_map", 0, 100, vehicle_map_message);
        for_planner_message = vehicle_map_message;
        for_planner_message.info.origin = message.pose.pose;
        pub_local_costmap_.publish(vehicle_map_message);
        pub_for_planner_.publish(for_planner_message);

        grid_map_msgs::GridMap for_planner_grid_map_message;
        grid_map::GridMapRosConverter::toMessage(vehicle_map_, for_planner_grid_map_message);
        pub_for_planner_grid_map_uncertainty_.publish(for_planner_grid_map_message);
        // ROS_INFO("1111111111111 %f %f %f", for_planner_message.info.origin.position.x, for_planner_message.info.origin.position.y, for_planner_message.info.origin.position.z);
        // ROS_INFO("2222222222222 %f %f %d %d", yaw/M_PI*180, for_planner_message.info.resolution, for_planner_message.info.width, for_planner_message.info.height);
        // ROS_INFO_THROTTLE(1.0, "vehicle grid map (timestamp %f) published.", vehicle_map_message.header.stamp.toSec());
    }

    void waypointsCallback(const nav_msgs::Path::ConstPtr &global_path)
    {
        waypoints_ptr = new nav_msgs::Path(*global_path);
        old_nearest_point_index = -1;
    }

    void lidarMapCallback(const nav_msgs::OccupancyGrid &message)
    {
        lidarRaw = message;
    }

    void bondingBoxCallback(const visualization_msgs::MarkerArray &markerArray)
    {
        bbox_ = markerArray;
    }

    void bboxCallback(const std_msgs::UInt8MultiArray &message)
    {
        ROS_INFO("1 %d 2 %d 3 %d 4 %d", message.data[0], message.data[1], message.data[2], message.data[3]);
        if (message.data[0] >= 150 || message.data[2] >= 150 || message.data[1] >= 100 || message.data[3] >= 100)
        {
            memset(formerX, 0, sizeof(int) * 4);
            vehicle_map_.clear("semantic_lidar_map");
            return;
        }

        int x_1 = 150 - message.data[0],
            y_1 = 100 - message.data[1],
            x_2 = 150 - message.data[2],
            y_2 = 100 - message.data[3];

        int x_center, y_center, width, height;

        // xyxy -> xywh
        width = message.data[3] - message.data[1];
        height = message.data[0] - message.data[2];
        x_center = 50 - message.data[3] + 0.5 * width;
        y_center = message.data[2] + 0.5 * height;

        // 2. kalman prediction
        Mat prediction = KF.predict();

        // ROS_INFO("former  %d, %d, %d, %d",formerX[0], formerX[1], formerX[2], formerX[3]);
        // ROS_INFO("predict %d, %d, %d, %d",(int)measurement.at<double>(0),(int)measurement.at<double>(1),(int)measurement.at<double>(2),(int)measurement.at<double>(3));
        // ROS_INFO("true    %d, %d, %d, %d",x_center, y_center, width, height);

        // Index submapStartIndex(x_1, y_2);
        Index submapStartIndex(150 - (int)measurement.at<double>(1) - 0.5 * (int)measurement.at<double>(3),
                               50 + (int)measurement.at<double>(0) - 0.5 * (int)measurement.at<double>(2));
        Index submapBufferSize((int)measurement.at<double>(3), (int)measurement.at<double>(2));
        Index temp_index;
        for (grid_map::SubmapIterator iterator(vehicle_map_, submapStartIndex, submapBufferSize);
             !iterator.isPastEnd(); ++iterator)
        {
            vehicle_map_.at("semantic_lidar_map", *iterator) = 100;
            Position temp;
            vehicle_map_.getPosition(*iterator, temp);
            vehicle_map_.getIndex(temp, temp_index);
            // ROS_INFO("x %d y %d", temp_index[0], temp_index[1]);
        }

        // 3.update measurement
        measurement.at<double>(0) = (double)x_center;
        measurement.at<double>(1) = (double)y_center;
        measurement.at<double>(2) = (double)width;
        measurement.at<double>(3) = (double)height;

        // 4.update
        KF.correct(measurement);

        // KalmanFilter KF(stateNum, measureNum, 0);

        for (int i = 0; i < 4; i++)
        {
            formerX[i] = (int)measurement.at<double>(i);
        }

        nav_msgs::OccupancyGrid map_message;
        GridMapRosConverter::toOccupancyGrid(vehicle_map_, "semantic_lidar_map", 0, 100, map_message);
        // visual_semantic_lidar_map_.publish(map_message);
        // vehicle_map_.clear("semantic_lidar_map");
        // visualMap("semantic_lidar_map");
    }

    void bboxFCallback(const std_msgs::Float64MultiArray &message)
    {
        bboxParam = message;
    }

    void vehicleInfoCallback(const carla_msgs::CarlaEgoVehicleInfo &message)
    {
        vehicle_id = to_string(message.id);
    }
    void staticobstacleCallback(const vehiclepub::VehicleInfoArray::ConstPtr &message)
    {
        this->msg_vehicles.vehicles = message->vehicles;
    }

    vector<double> getConfidenceEllipse(Eigen::Matrix2f cov, double chisquare_val = 2.4477)
    {
        vector<double> params;

        Eigen::EigenSolver<Eigen::Matrix2f> es(cov);

        // Get the eigenvalues and eigenvectors
        Eigen::Matrix2f D = es.pseudoEigenvalueMatrix();
        Eigen::Matrix2f V = es.pseudoEigenvectors();

        // find the bigger eigen value
        int major_index, minor_index;

        if (D(0, 0) > D(1, 1))
        {
            major_index = 0;
            minor_index = 1;
        }
        else
        {
            major_index = 1;
            minor_index = 0;
        }

        // Calculate the angle between the largest eigenvector and the x-axis
        double angle = atan2(V(major_index, 1), V(major_index, 0));
        if (angle < 0)
        {
            angle += 6.28318530718;
        }

        // Calculate the size of the minor and major axes
        double half_major_axis = chisquare_val * sqrt(D(major_index, major_index));
        double half_minor_axis = chisquare_val * sqrt(D(minor_index, minor_index));

        // cout << D << endl;
        // cout << V << endl;

        params.push_back(half_major_axis);
        params.push_back(half_minor_axis);
        params.push_back(angle);
        // if (sqrt(D(minor_index,minor_index)) == )
        // ROS_INFO("angle %f, major %f, minor %f", angle, sqrt(D(major_index,major_index)), sqrt(D(minor_index,minor_index)));
        return params;
    }

    void visualGlobalMap()
    {
        ros::Time time = ros::Time::now();
        global_map_.setTimestamp(time.toNSec());
        nav_msgs::OccupancyGrid global_map_message;
        GridMapRosConverter::toOccupancyGrid(global_map_, "global_map", 0, 100, global_map_message);
        pub_global_costmap_.publish(global_map_message);
    }

    void visualMap(string layer)
    {
        vehicle_map_.setTimestamp(ros::Time::now().toNSec());
        nav_msgs::OccupancyGrid map_message;
        GridMapRosConverter::toOccupancyGrid(vehicle_map_, layer, 0, 100, map_message);
        visual_vehicle_map_.publish(map_message);
        // vehicle_map_.clear(layer);
    }

    void propagateUncertainty(int index, double sin_Vtheta_og, double cos_Vtheta_og, double Cx, double Cy)
    {
        bool ab = true;
        // clock_t begin_timer = clock();
        double start_time = omp_get_wtime();

        if (ab)
        {
            vector<pair<int, double>> results = thrust_propagateUncertainty(vehicle_map_, index, sin_Vtheta_og, cos_Vtheta_og, sigma_x, sigma_y, sigma_theta);

            int i = 0;
            for (GridMapIterator it(vehicle_map_); !it.isPastEnd(); ++it, ++i)
            {
                int count = results[i].first;
                double result = results[i].second;
                if (0 == count)
                {
                    vehicle_map_.at("uncertainty_map", *it) = vehicle_map_.at("vehicle_map", *it);
                    continue;
                }

                vehicle_map_.at("uncertainty_map", *it) = result;
            }
        }

        if (!ab)
        {
            int count1 = 0;
            // double numerator = 0;
            // double denominator = 0;
            // double sigma_x_i, sigma_y_i;
            // double u, v, t, rho;
            vector<double> ellipse_params;
            GridMapIterator it(vehicle_map_);
            vector<double> times{0.0, 0.0, 0.0, 0.0};
            clock_t m_timer = clock();

            for (unsigned int i = 0; i < index; ++i)
            {
                ++it;
            }

            int count2 = 0;
            for (; !it.isPastEnd(); ++it)
            {

                if (count1++ == (((X_LENGTH * Y_LENGTH) / (RESOLUTION * RESOLUTION)) / THREADS_NUM))
                    break;

                double numerator = 0;
                double denominator = 0;
                double sigma_x_i, sigma_y_i;
                double u, v, t, rho;

                // numerator = 0;
                // denominator = 0;
                Position position;
                vehicle_map_.getPosition(*it, position);
                double Cx = position.x();
                double Cy = position.y();
                // ROS_INFO("Cx %f Cy %f", Cx, Cy);

                u = (-sin_Vtheta_og * Cx - cos_Vtheta_og * Cy) * (-sin_Vtheta_og * Cx - cos_Vtheta_og * Cy);
                v = (cos_Vtheta_og * Cx - sin_Vtheta_og * Cy) * (cos_Vtheta_og * Cx - sin_Vtheta_og * Cy);
                t = sin_Vtheta_og * cos_Vtheta_og * (Cx * Cx - Cy * Cy) + Cx * Cy * (sin_Vtheta_og * sin_Vtheta_og - cos_Vtheta_og * cos_Vtheta_og);

                sigma_x_i = sqrt(sigma_x * sigma_x + sigma_theta * sigma_theta * u);
                sigma_y_i = sqrt(sigma_y * sigma_y + sigma_theta * sigma_theta * v);
                rho = sigma_theta * sigma_theta * t / (sigma_x_i * sigma_y_i);

                // if (count1 == 1){
                //     ROS_INFO("(Cx_1, Cy_1):(%lf, %lf)",Cx, Cy);
                //     ROS_INFO("1.(u, v, t):(%lf, %lf, %lf)", u, v, t);
                //     ROS_INFO("1.(sigma_x_i, sigma_y_i, rou):(%lf, %lf, %lf)", sigma_x_i, sigma_y_i, rho);
                // }
                times[0] += (double)(clock() - m_timer) / CLOCKS_PER_SEC;
                m_timer = clock();

                Eigen::Matrix2f cov_i;
                double a, b, c;
                a = sigma_x_i * sigma_x_i;
                b = rho * sigma_x_i * sigma_y_i;
                c = sigma_y_i * sigma_y_i;
                cov_i << a, b, b, c;
                // ROS_INFO("%f, %f, %f", a, b, c);
                // if (count2 < 3){
                //     ROS_INFO("a %f b %f c %f", a, b, c);
                //     count2++;
                // }

                ellipse_params = getConfidenceEllipse(cov_i);
                // count2++;
                // if(count1 < 4){
                //     // ROS_INFO("cov_i:(%lf. %lf, %lf, %lf)", cov_i(0,0), cov_i(0,1), cov_i(1,0), cov_i(1,1));
                //     ROS_INFO("params1:%lf, params2:%lf, params3:%lf", ellipse_params[0], ellipse_params[1], ellipse_params[2]);
                // }

                times[1] += (double)(clock() - m_timer) / CLOCKS_PER_SEC;
                m_timer = clock();

                // optimation computation time
                // bool skip = false;

                // for (grid_map::EllipseIterator iterator(vehicle_map_, position, Length(2*ellipse_params[0],2*ellipse_params[1]), ellipse_params[2]);
                //     !iterator.isPastEnd(); ++iterator) {
                //     if (vehicle_map_.at("vehicle_map", *iterator) != 0) {
                //         skip = true;
                //     }
                // }

                // if (skip) {
                //     vehicle_map_.at("uncertainty_map", *it) = vehicle_map_.at("vehicle_map", *it);
                //     continue;
                // }

                // ROS_INFO("u1 %f u2 %f o1 %f o2 %f rho %f", Cx, Cy, sigma_x_i, sigma_y_i, rho);

                int count = 0;
                // double calcu = 0;

                // double result = thrust_ellipseProbability(vehicle_map_, position, ellipse_params, Cx, Cy, sigma_x_i, sigma_y_i, rho);

                for (grid_map::EllipseIterator iterator(vehicle_map_, position, Length(2 * ellipse_params[0], 2 * ellipse_params[1]), ellipse_params[2]);
                     !iterator.isPastEnd(); ++iterator)
                {
                    double f_i;
                    double Cxj, Cyj;

                    Position position_j;
                    vehicle_map_.getPosition(*iterator, position_j);
                    Cxj = position_j.x();
                    Cyj = position_j.y();

                    // f_i = exp(-(pow((Cxj-Cx) / sigma_x_i, 2) - 2 * rho * ((Cxj-Cx) / sigma_x_i) * ((Cyj-Cy) / sigma_y_i) + pow((Cyj-Cy) / sigma_y_i, 2))
                    //     / (2 * (1 - pow(rho, 2))))
                    //     / (2 * M_PI * sigma_x_i * sigma_y_i * sqrt(1 - pow(rho, 2)));

                    f_i = nomal2(Cxj, Cyj, Cx, Cy, sigma_x_i, sigma_y_i, rho);

                    if (f_i == 0)
                    {
                        // ROS_INFO("%lf", f_i);
                    }

                    numerator += f_i * vehicle_map_.atPosition("vehicle_map", position_j);
                    denominator += f_i;
                    count++;
                    // calcu += f_i;
                    // vehicle_map_.at("ellipse_map", *iterator) = 0;
                }

                times[2] += (double)(clock() - m_timer) / CLOCKS_PER_SEC;
                m_timer = clock();

                if (0 == count)
                {
                    vehicle_map_.at("uncertainty_map", *it) = vehicle_map_.at("vehicle_map", *it);
                    continue;
                }

                // Index temp_index;
                // vehicle_map_.getIndex(position, temp_index);

                // ROS_INFO("x %d, y %d", temp_index[0], temp_index[1]);
                // visualMap("ellipse_map");

                // ROS_INFO("count %d", count);
                // ROS_INFO("count1 %d, %f", count1, calcu);
                // ROS_INFO("numerator %f, denominator %f, uncertainty %f", numerator, denominator, numerator / denominator);
                // if (denominator == 0) {
                //     vehicle_map_.at("uncertainty_map", *it) = vehicle_map_.atPosition("vehicle_map", position);
                // }

                vehicle_map_.at("uncertainty_map", *it) = numerator / denominator;
            }
            ROS_INFO("times[0] %f times[1] %f times[2] %f times[3] %f", times[0], times[1], times[2], times[3]);
        }

        double end_time = omp_get_wtime();
        std_msgs::Int32 compute_time;
        compute_time.data = (int)((end_time - start_time) * 1000);
        pub_compute_time_.publish(compute_time);

        // ROS_INFO("sum_time:%lf", (double)(clock()-begin_timer)/CLOCKS_PER_SEC);
        ROS_INFO("sum_time:%lf", end_time - start_time);
    }

    void searchTargetIndex(double Vx_og, double Vy_og)
    {
        if (waypoints_ptr == nullptr)
        {
            ROS_INFO("not receive waypoints");
            return;
        }
        int waypoints_num = waypoints_ptr->poses.size();
        double disMin = INT_MAX;
        if (old_nearest_point_index == -1)
        {
            for (int i = 0; i < waypoints_num; i++)
            {
                double dx = waypoints_ptr->poses[i].pose.position.x - Vx_og,
                       dy = waypoints_ptr->poses[i].pose.position.y - Vy_og;
                double temp_dis = sqrt(dx * dx + dy * dy);
                if (temp_dis < disMin)
                {
                    disMin = temp_dis;
                    old_nearest_point_index = i;
                }
            }
        }
        else
        {
            int ind = old_nearest_point_index;
            double dx = waypoints_ptr->poses[ind].pose.position.x - Vx_og,
                   dy = waypoints_ptr->poses[ind].pose.position.y - Vy_og;
            double distance_this_index = sqrt(dx * dx + dy * dy);
            double distance_next_index;
            while (true)
            {
                dx = waypoints_ptr->poses[ind + 1].pose.position.x - Vx_og,
                dy = waypoints_ptr->poses[ind + 1].pose.position.y - Vy_og;
                distance_next_index = sqrt(dx * dx + dy * dy);
                if (distance_this_index < distance_next_index)
                    break;
                if (ind + 1 < waypoints_num)
                {
                    ind = ind + 1;
                }
                else
                {
                    ind;
                }
                distance_this_index = distance_next_index;
            }
            old_nearest_point_index = ind;
        }
    }

    void getVehicleMapScale()
    {
        // 1. get waypoints' pose
        int waypoints_num = waypoints_ptr->poses.size();
        int end_waypoints_num = old_nearest_point_index + LOOK_AHEAD_DISTENCE;
        if (waypoints_num - old_nearest_point_index < LOOK_AHEAD_DISTENCE)
        {
            end_waypoints_num = waypoints_num - 1;
        }
        // 2. get corridor coordinate in global
        vector<geometry_msgs::PoseStamped> corridor_left, corridor_right;
        double center_x, center_y, center_heading;
        for (int i = old_nearest_point_index; i < end_waypoints_num; i++)
        {

            center_x = waypoints_ptr->poses[i].pose.position.x;
            center_y = waypoints_ptr->poses[i].pose.position.y;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(waypoints_ptr->poses[i].pose.orientation, quat);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            center_heading = yaw - M_PI_2;
            if (center_heading < 0)
            {
                center_heading += 2 * M_PI;
            }
            double left_corridor_x, left_corridor_y;
            left_corridor_x = center_x - 8 * cos(center_heading);
            left_corridor_y = center_y - 8 * sin(center_heading);
            geometry_msgs::PoseStamped left_corridor = waypoints_ptr->poses[i];
            left_corridor.header.frame_id = "map";
            left_corridor.pose.position.x = left_corridor_x;
            left_corridor.pose.position.y = left_corridor_y;
            corridor_left.push_back(left_corridor);
            // right corridor
            double right_corridor_x, right_corridor_y;
            right_corridor_x = center_x + 4 * cos(center_heading);
            right_corridor_y = center_y + 4 * sin(center_heading);
            geometry_msgs::PoseStamped right_corridor = waypoints_ptr->poses[i];
            right_corridor.header.frame_id = "map";
            right_corridor.pose.position.x = right_corridor_x;
            right_corridor.pose.position.y = right_corridor_y;
            corridor_right.push_back(right_corridor);
        }
        // 3. corridor coordinate global -> vehicle
        double x_max = DBL_MIN,
               x_min = DBL_MAX,
               y_max = DBL_MIN,
               y_min = DBL_MAX;
        for (int i = 0; i < corridor_left.size(); i++)
        {
            corridor_left[i] = tf_buffer.transform(corridor_left[i], vehicle_id);
            corridor_right[i] = tf_buffer.transform(corridor_right[i], vehicle_id);
            if (corridor_left[i].pose.position.x > x_max)
                x_max = corridor_left[i].pose.position.x;
            if (corridor_left[i].pose.position.x < x_min)
                x_min = corridor_left[i].pose.position.x;
            if (corridor_left[i].pose.position.y > y_max)
                y_max = corridor_left[i].pose.position.y;
            if (corridor_left[i].pose.position.y < y_min)
                y_min = corridor_left[i].pose.position.y;
            if (corridor_right[i].pose.position.x > x_max)
                x_max = corridor_right[i].pose.position.x;
            if (corridor_right[i].pose.position.x < x_min)
                x_min = corridor_right[i].pose.position.x;
            if (corridor_right[i].pose.position.y > y_max)
                y_max = corridor_right[i].pose.position.y;
            if (corridor_right[i].pose.position.y < y_min)
                y_min = corridor_right[i].pose.position.y;
        }
        // 4. get scale of the vehicle map
        X_LENGTH = x_max - x_min;
        Y_LENGTH = y_max - y_min;
        X_POSITION = X_LENGTH / 2;
        // if ((center_heading > 3/4*M_PI && center_heading < 5/4*M_PI) ||
        //     center_heading < 1/4*M_PI || center_heading > 7/4*M_PI)
        // {
        //     right_corridor_y = -right_corridor_y;
        // }
        Y_POSITION = (y_max + y_min) / 2;
        // publish vehicle map scale
        map_engine::map_param new_param;
        new_param.x_length = X_LENGTH;
        new_param.y_length = Y_LENGTH;
        new_param.x_position = X_POSITION;
        new_param.y_position = Y_POSITION;
        new_param.resolution = RESOLUTION;
        pub_map_param_.publish(new_param);

        // data for experiment
        std_msgs::Int32 grid_num;
        grid_num.data = (X_LENGTH / RESOLUTION) * (Y_LENGTH / RESOLUTION);
        pub_grid_num_.publish(grid_num);
    }

    void bondingBoxHandle1()
    {
        grid_map::Polygon polygon;
        polygon.setFrameId(vehicle_map_.getFrameId());
        polygon.addVertex(Position(bboxParam.data[0], bboxParam.data[1]));
        polygon.addVertex(Position(bboxParam.data[2], bboxParam.data[3]));
        polygon.addVertex(Position(bboxParam.data[4], bboxParam.data[5]));
        polygon.addVertex(Position(bboxParam.data[6], bboxParam.data[7]));
        polygon.addVertex(Position(bboxParam.data[0], bboxParam.data[1]));

        for (grid_map::PolygonIterator iterator(vehicle_map_, polygon);
             !iterator.isPastEnd(); ++iterator)
        {
            vehicle_map_.at("semantic_lidar_map", *iterator) = 100;
        }

        nav_msgs::OccupancyGrid map_message;
        GridMapRosConverter::toOccupancyGrid(vehicle_map_, "semantic_lidar_map", 0, 100, map_message);
        visual_semantic_lidar_map_.publish(map_message);
    }

    void bondingBoxHandle()
    {

        for (auto marker : bbox_.markers)
        {
            grid_map::Polygon polygon;
            polygon.setFrameId(vehicle_map_.getFrameId());
            double minX, minY, maxX, maxY;

            minX = marker.pose.position.x - marker.scale.x / 2;
            maxX = marker.pose.position.x + marker.scale.x / 2;
            minY = marker.pose.position.y - marker.scale.y / 2;
            maxY = marker.pose.position.y + marker.scale.y / 2;

            polygon.addVertex(Position(minX, maxY));
            polygon.addVertex(Position(maxX, maxY));
            polygon.addVertex(Position(maxX, minY));
            polygon.addVertex(Position(minX, minY));
            polygon.addVertex(Position(minX, maxY));

            for (grid_map::PolygonIterator iterator(vehicle_map_, polygon);
                 !iterator.isPastEnd(); ++iterator)
            {
                vehicle_map_.at("bounding_box_map", *iterator) = 100;
            }
        }

        // nav_msgs::OccupancyGrid map_message;
        // GridMapRosConverter::toOccupancyGrid(vehicle_map_, "bounding_box_map", 0, 100, map_message);
        // visual_semantic_lidar_map_.publish(map_message);
    }

    void bondingBoxHandle(const vehiclepub::VehicleInfoArray &vehicles, grid_map::GridMap &vehicle_map_,
                          const double own_pos_x, const double own_pos_y, const double own_yaw)
    {

        for (const auto &vehicle_info : vehicles.vehicles)
        {
            double posX = vehicle_info.pose.position.x;
            double posY = -vehicle_info.pose.position.y; // Assuming the y-coordinate doesn't need to be negated

            // Calculate the distance from the current position
            double distance = std::sqrt(std::pow(posX - own_pos_x, 2) + std::pow(posY - own_pos_y, 2));
            // ROS_INFO("Vehicle pos %.2f , %.2f",own_pos_x,own_pos_y);
            // ROS_INFO("Obstacle pos %.2f , %.2f",posX,posY);
            // ROS_INFO("distance is %.2f",distance);
            // Filter obstacles within 50 units of the current position
            if (distance <= 100.0)
            {
                grid_map::Polygon polygon;
                polygon.setFrameId(vehicle_map_.getFrameId());

                double sizeX = vehicle_info.size.x + 0.2;
                double sizeY = vehicle_info.size.y + 0.2;
                double yaw = -vehicle_info.pose.orientation.z;

                // Calculate half dimensions
                double half_sizeX = sizeX / 2.0;
                double half_sizeY = sizeY / 2.0;

                // Calculate the four corners relative to the vehicle's center
                std::vector<grid_map::Position> corners = {
                    {half_sizeX, half_sizeY},
                    {half_sizeX, -half_sizeY},
                    {-half_sizeX, -half_sizeY},
                    {-half_sizeX, half_sizeY}};

                // Rotate and translate the corners
                for (auto &corner : corners)
                {
                    // Rotate by vehicle's own yaw angle
                    double global_x = cos(yaw) * corner.x() - sin(yaw) * corner.y() + posX;
                    double global_y = sin(yaw) * corner.x() + cos(yaw) * corner.y() + posY;

                    // Translate to the local coordinate system
                    double local_x = cos(own_yaw) * (global_x - own_pos_x) + sin(own_yaw) * (global_y - own_pos_y);
                    double local_y = -sin(own_yaw) * (global_x - own_pos_x) + cos(own_yaw) * (global_y - own_pos_y);

                    // Print the transformed coordinates
                    // ROS_INFO("Transformed corner coordinates: (%.2f, %.2f)", local_x, local_y);

                    polygon.addVertex(grid_map::Position(local_x, local_y));
                }

                // Close the polygon by adding the first vertex again
                polygon.addVertex(polygon.getVertex(0));

                // Fill the grid map with the bounding box
                for (grid_map::PolygonIterator iterator(vehicle_map_, polygon); !iterator.isPastEnd(); ++iterator)
                {
                    vehicle_map_.at("bounding_box_map", *iterator) = 100;
                }
            }
        }
    }

private:
    //! ROS nodehandle.
    ros::NodeHandle &nodeHandle_;

    // subscriber
    ros::Subscriber sub_global_map_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_lidar_;
    ros::Subscriber sub_bbox_;
    ros::Subscriber sub_bbox_f_;
    ros::Subscriber sub_bonding_box_;
    ros::Subscriber sub_waypoints_;
    ros::Subscriber sub_lidar_obstacle_;
    ros::Subscriber sub_vehicle_info_;
    ros::Subscriber sub_static_obstacle_;

    // publisher
    ros::Publisher pub_local_costmap_;
    ros::Publisher pub_global_costmap_;
    ros::Publisher pub_for_planner_;
    ros::Publisher pub_map_param_;
    ros::Publisher pub_grid_num_;
    ros::Publisher pub_compute_time_;

    ros::Publisher pub_for_planner_grid_map_uncertainty_;

    // visualization for Rviz
    ros::Publisher visual_vehicle_map_;
    ros::Publisher visual_semantic_lidar_map_;

    // grid map
    grid_map::GridMap global_map_;
    grid_map::GridMap vehicle_map_;

    // laneInfo
    nav_msgs::Path *waypoints_ptr;
    int old_nearest_point_index;

    // temp variaties
    std_msgs::Float64MultiArray bboxParam;
    nav_msgs::OccupancyGrid lidarRaw;
    visualization_msgs::MarkerArray bbox_;

    // vehicle info
    string vehicle_id;

    // dynamic reconfigure
    dynamic_reconfigure::Server<map_engine::map_engine_Config> server;
    dynamic_reconfigure::Server<map_engine::map_engine_Config>::CallbackType f;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // uncertainty
    double sigma_x;
    double sigma_y;
    double sigma_theta;

    // kalman filter
    int stateNum;
    int measureNum;
    int formerX[4];
    KalmanFilter KF;
    // Mat processNoise(stateNum, 1, CV_32F);
    Mat measurement;

    vehiclepub::VehicleInfoArray msg_vehicles;
};

int main(int argc, char **argv)
{
    // Initialize node and subscriber/publisher.
    ros::init(argc, argv, "local_costmap");
    ros::NodeHandle nh("~");
    LocalCostmap localCostmap(nh);
    ros::spin();

    return 0;
}
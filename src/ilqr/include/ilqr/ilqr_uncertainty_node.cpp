#include "ilqr_uncertainty_node.h"



/*
Function:initial and connect.
*/
ilqr_uncertainty_node::ilqr_uncertainty_node() : tf_listener(tf_buffer), ilqrplanner(params)
{
    current_ego_vehicle_state = Eigen::VectorXd::Zero(4);
    current_ego_vehicle_state_noise = Eigen::VectorXd::Zero(4);

    this->lane_info_sub = nh.subscribe("/carla/ego_vehicle/waypoints", 1, &ilqr_uncertainty_node::laneInfoCallback, this);
    this->odom_sub = nh.subscribe("/carla/ego_vehicle/odometry", 1, &ilqr_uncertainty_node::odomCallback, this);
    // this->static_obstacle_sub = nh.subscribe("/static_obstacle/vehicle_info",1, &ilqr_uncertainty_node::staticobstacleCallback, this);
    this->map_sub = nh.subscribe("local_costmap/for_planner_map", 1, &ilqr_uncertainty_node::mapCallback, this);
    this->map_param_sub = nh.subscribe("local_costmap/map_param", 1, &ilqr_uncertainty_node::mapParamCallback, this);
    this->grid_map_sub = nh.subscribe("local_costmap/for_planner_map_grid_map", 1, &ilqr_uncertainty_node::gridmapCallback, this);

    this->ilqr_path_pub = nh.advertise<visualization_msgs::MarkerArray>("ILQR_Path", 10);
    this->vehicle_cmd_pub = nh.advertise<ackermann_msgs::AckermannDrive>("/carla/ego_vehicle/ackermann_cmd", 1);
    this->experiment_data_pub = nh.advertise<vehiclepub::Experiment>("experiment",10);

    // this->client.getCurrentConfiguration(config);  // 获取当前配置

    // this->SIGMA_X = 0.02;
    // this->SIGMA_Y = 0.02;
    // this->SIGMA_THETA = 0.05;
    nh.getParam("sigma_x",this->SIGMA_X);
    nh.getParam("sigma_y",this->SIGMA_Y);
    nh.getParam("sigma_theta",this->SIGMA_THETA);
    nh.getParam("safe_length",this->params.safe_length);
    nh.getParam("safe_width",this->params.safe_width);
    nh.getParam("w_uncertainty",this->params.w_uncertainty);

    
    // this->X_LENGTH = config.x_length;
    // this->Y_LENGTH = config.y_length;
    // this->X_POSITION = config.x_position;
    // this->Y_POSITION = config.y_position;
    // this->RESOLUTION = config.resolution;
}
/*
Function: get global path in global position. only once
*/
void ilqr_uncertainty_node::laneInfoCallback(const nav_msgs::Path::ConstPtr &msg)
{
    global_path.resize(2, msg->poses.size()); // In global position,global path waypoints
    for (int i = 0; i < msg->poses.size(); i++)
    {
        // ROS_INFO("Received odometry message: Position(x=%f, y=%f, z=%f), Orientation(x=%f, y=%f, z=%f, w=%f)",
        //          msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, msg->poses[i].pose.position.z,
        //          msg->poses[i].pose.orientation.x, msg->poses[i].pose.orientation.y, msg->poses[i].pose.orientation.w);
        // ROS_INFO("Received odometry message: Position(x=%f, y=%f, z=%f)",
        //          msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, msg->poses[i].pose.position.z);
        global_path(0, i) = msg->poses[i].pose.position.x;
        global_path(1, i) = msg->poses[i].pose.position.y;
    }
    ilqrplanner.set_global_plan(global_path);
}
/*
Function: get ego vehicle posistion and velocity, and plan path and publish.
*/
void ilqr_uncertainty_node::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    double current_state_x, current_state_y, current_state_v, current_state_yaw;

    // current_state_v = magnitude(odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.z);
    current_state_v = sqrt(odom_msg->twist.twist.linear.x * odom_msg->twist.twist.linear.x +
                           odom_msg->twist.twist.linear.y * odom_msg->twist.twist.linear.y +
                           odom_msg->twist.twist.linear.z * odom_msg->twist.twist.linear.z);
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer.lookupTransform("map", odom_msg->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> distribution_x(0, SIGMA_X);
    std::normal_distribution<double> distribution_y(0, SIGMA_Y);
    std::normal_distribution<double> distribution_theta(0, SIGMA_THETA);
    double rand_x = distribution_x(gen);
    double rand_y = distribution_y(gen);
    double rand_theta = distribution_theta(gen);
    // std::cout<<SIGMA_X<<SIGMA_Y<<SIGMA_THETA<<std::endl;

    // std::cout<<rand_x<<rand_y<<rand_theta<<std::endl;

    geometry_msgs::Pose pose_in_map;
    tf2::doTransform(odom_msg->pose.pose, pose_in_map, transform_stamped);
    // Current XY of robot (map frame)
    // current_state_.x = pose_in_map.position.x + rand_x;
    // current_state_.y = pose_in_map.position.y + rand_y;
    current_state_x = pose_in_map.position.x;
    current_state_y = pose_in_map.position.y;
    // map_height_ = pose_in_map.position.z - 0.3; // minus the tire radius

    tf2::Quaternion q_tf2(pose_in_map.orientation.x, pose_in_map.orientation.y,
                          pose_in_map.orientation.z, pose_in_map.orientation.w);
    tf2::Matrix3x3 m(q_tf2.normalize());
    double roll, pitch;
    m.getRPY(roll, pitch, current_state_yaw);

    current_ego_vehicle_state << current_state_x, current_state_y, current_state_v, current_state_yaw;
    current_ego_vehicle_state_noise << current_state_x + rand_x, current_state_y + rand_y, current_state_v, current_state_yaw + rand_theta;
    Uncertainty vehicle_map(this->params,this->map_msg,this->grid_map_msg,this->x_center,this->y_center,\
                            SIGMA_X,SIGMA_Y,SIGMA_THETA,0,0,this->nh);
    ilqrplanner.set_uncertainty_map(vehicle_map);

    ilqrplanner.set_global_plan(global_path);
    ros::Time begin_time=ros::Time::now();
    auto start_time = std::chrono::high_resolution_clock::now();

    ilqrplanner.run_step(current_ego_vehicle_state_noise);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    double planning_time = duration.count();
    // ilqrplanner.clear_Obstacle();
    ROS_INFO("Planning finished,use time %f s",duration);

    PathVisualization(ilqrplanner.X_result);
    // PathVisualization(ilqrplanner.ref_traj_result);
    // publishVehicleCmd(current_state_v, current_state_yaw, ilqrplanner.U_result(0, 1), ilqrplanner.U_result(1, 1));
    publishVehicleCmd(current_state_v, current_state_yaw, ilqrplanner.U_result(0, 0), ilqrplanner.U_result(1, 0));
    publishExperimentData(begin_time,current_ego_vehicle_state,planning_time,ilqrplanner.X_result,ilqrplanner.U_result);
    // std::cout<<"compute reference path in vehicle axis"<<std::endl;
    // std::cout<<ilqrplanner.ref_traj_result<<std::endl;
}

void ilqr_uncertainty_node::mapCallback(const nav_msgs::OccupancyGrid &message)
{
    this->map_msg = message;
}

void ilqr_uncertainty_node::mapParamCallback(const map_engine::map_param &message)
{
    this->x_center = message.x_position;
    this->y_center = message.y_position;
}

void ilqr_uncertainty_node::gridmapCallback(const grid_map_msgs::GridMap &message)
{
    this->grid_map_msg = message;
}

// void ilqr_uncertainty_node::staticobstacleCallback(const vehiclepub::VehicleInfoArray::ConstPtr &message)
// {
//     std::vector<Obstacle> static_obstalce;
//     static_obstalce.reserve(message->vehicles.size());
//     for (const auto &vehicle_info : message->vehicles)
//     {
//         Eigen::MatrixXd dimension = Eigen::MatrixXd::Zero(2, this->params.horizon);
//         Eigen::MatrixXd position = Eigen::MatrixXd::Zero(4, this->params.horizon);

//         const geometry_msgs::Quaternion &orientation = vehicle_info.pose.orientation;

//         // Convert quaternion to tf::Quaternion
//         tf::Quaternion quat;
//         tf::quaternionMsgToTF(orientation, quat);

//         // Convert tf::Quaternion to tf::Matrix3x3
//         tf::Matrix3x3 mat(quat);

//         // Convert to roll, pitch, yaw (in radians)
//         double roll, pitch, yaw;
//         mat.getRPY(roll, pitch, yaw);

//         // ROS_INFO("Orientation: roll: %.2f, pitch: %.2f, yaw: %.2f", roll, pitch, yaw);

//         for (int i = 0; i < this->params.horizon; i++)
//         {
//             dimension(0, i) = vehicle_info.size.x;
//             dimension(1, i) = vehicle_info.size.y;

//             position(0, i) = vehicle_info.pose.position.x;
//             position(1, i) = -vehicle_info.pose.position.y;
//             position(2, i) = 0;
//             position(3, i) = yaw;
//             // position(3,i) = (vehicle_info.pose.orientation.z + 360.0)%360/180.0*3.1415926;
//         }
//         Obstacle temp(this->params, dimension, position);
//         static_obstalce.push_back(temp);
//     }
//     this->ilqrplanner.set_Obstacle(static_obstalce);
// }
/*
Function:PathVisualization.
*/
void ilqr_uncertainty_node::PathVisualization(MatrixXd &path)
{

    visualization_msgs::MarkerArray path_markerarray;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;  // 矩形的宽度
    marker.scale.y = 0.1;  // 矩形的长度
    marker.scale.z = 0.01; // 矩形的高度
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(1.0); // 显示时间为1秒
    for (int i = 0; i < path.cols(); i++)
    {
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        marker.pose.position.x = path(0, i);
        marker.pose.position.y = path(1, i);
        marker.pose.position.z = 0.0;
        path_markerarray.markers.emplace_back(marker);
    }
    this->ilqr_path_pub.publish(path_markerarray);
}
/*
Function:Publish cmd topic.
*/
void ilqr_uncertainty_node::publishVehicleCmd(const double &speed, const double &angle, const double &accel, const double &angle_velocity)
{
    ackermann_msgs::AckermannDrive vehicle_cmd;
    vehicle_cmd.steering_angle = angle_velocity; // rad
    vehicle_cmd.steering_angle_velocity = 0;     // rad/s
    vehicle_cmd.speed = speed + accel;           // m/s
    vehicle_cmd.acceleration = 0;                // m/s^2
    vehicle_cmd.jerk = 0;
    vehicle_cmd_pub.publish(vehicle_cmd);
}

/*
Function:Publish cmd topic.
*/
void ilqr_uncertainty_node::publishExperimentData(ros::Time start_time, 
                             const VectorXd &start_pos, 
                             const double &planning_time, 
                             const MatrixXd &X, 
                             const MatrixXd &U )
{
    vehiclepub::Experiment msg;
    msg.start_time = start_time;
    msg.planning_time = planning_time;

    // 使用 std::vector 初始化
    std::vector<double> vector_start_pos(4);
    std::vector<double> vector_x(4 * (U.cols() + 1));
    std::vector<double> vector_u(2 * U.cols());

    // 填充 vector_start_pos
    vector_start_pos[0] = start_pos(0);
    vector_start_pos[1] = start_pos(1);
    vector_start_pos[2] = start_pos(2);
    vector_start_pos[3] = start_pos(3);

    // 填充 vector_x 和 vector_u
    for (int i = 0; i < U.cols() + 1; ++i) {
        vector_x[4 * i]     = X(0, i);
        vector_x[4 * i + 1] = X(1, i);
        vector_x[4 * i + 2] = X(2, i);
        vector_x[4 * i + 3] = X(3, i);
    }
    for (int i = 0; i < U.cols(); ++i) {
        vector_u[2 * i]     = U(0, i);
        vector_u[2 * i + 1] = U(1, i);
    }

    // 将填充好的数据赋值给 msg
    msg.start_pos = vector_start_pos;
    msg.X = vector_x;
    msg.U = vector_u;

    // 发布消息
    this->experiment_data_pub.publish(msg);
}


MatrixXd ilqr_uncertainty_node::pathInGlobal2Vechicle(const MatrixXd &global_path, const VectorXd &ego_state)
{
    MatrixXd vehicle_path = MatrixXd::Zero(2, global_path.cols());
    double cos_theta = std::cos(ego_state(3));
    double sin_theta = std::sin(ego_state(3));

    // ROS_INFO("global_path cols is %d",global_path.cols());
    for (int i = 0; i < global_path.cols(); i++)
    {
        vehicle_path(0, i) = (global_path(0, i) - ego_state(0)) * cos_theta + (global_path(1, i) - ego_state(1)) * sin_theta; // x
        vehicle_path(1, i) = (global_path(0, i) - ego_state(0)) * sin_theta - (global_path(1, i) - ego_state(1)) * cos_theta; // y
    }
    return vehicle_path;
}

MatrixXd ilqr_uncertainty_node::pathInVechicle2Global(const MatrixXd &vehicle_path, const VectorXd &ego_state)
{
    MatrixXd global_path = MatrixXd::Zero(2, vehicle_path.cols());
    double cos_theta = std::cos(ego_state(3));
    double sin_theta = std::sin(ego_state(3));
    // ROS_INFO("vehicle_path cols is %d",vehicle_path.cols());
    for (int i = 0; i < vehicle_path.cols(); i++)
    {
        global_path(0, i) = ego_state(0) + vehicle_path(0, i) * cos_theta + vehicle_path(1, i) * sin_theta;
        global_path(1, i) = ego_state(1) + vehicle_path(0, i) * sin_theta - vehicle_path(1, i) * cos_theta;
    }
    return global_path;
}
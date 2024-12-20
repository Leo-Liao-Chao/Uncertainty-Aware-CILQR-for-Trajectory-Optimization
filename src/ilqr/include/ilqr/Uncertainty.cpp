#include "Uncertainty.h"

Uncertainty::Uncertainty()
{
}

Uncertainty::Uncertainty(const Parameters &param, const nav_msgs::OccupancyGrid &map, const double x_center, const double y_center,
                         double sigma_X, double sigma_Y, double sigma_Theta, double sigma_Accelerate, double sigma_AngleVelocity)
{
    this->param = param;
    this->map = map;
    this->x_center = x_center;
    this->y_center = y_center;

    this->sigma_X = sigma_X;
    this->sigma_Y = sigma_Y;
    this->sigma_Theta = sigma_Theta;
    this->sigma_Accelerate = sigma_Accelerate;
    this->sigma_AngleVelocity = sigma_AngleVelocity;

    this->uncertainty_cost.setZero();
    uncertainty_cost(0,0) = this->param.w_uncertainty;
    uncertainty_cost(1,1) = this->param.w_uncertainty;

    this->PX.setZero();
    PX(0)=1;
    this->PY.setZero();
    PY(1)=1;

    //grid map
    bool convert_flag=false;
    convert_flag=grid_map::GridMapRosConverter::fromOccupancyGrid(map,"uncertainty_map",this->gridmap_uncertaintymap);
    // ROS_INFO("Occupancy Map msg convert!");
    // ROS_INFO("Covert successfull(1/0) %d",convert_flag);

    InitialUncertaintyMap();

    this->vehicle_length_half = this->param.length/2.0;//1.5
    this->vehicle_width_half = this->param.width/2.0;//1.5
}

Uncertainty::Uncertainty(const Parameters &param, const nav_msgs::OccupancyGrid &map, const grid_map_msgs::GridMap &grid_map, const double x_center, const double y_center, 
            double sigma_X, double sigma_Y, double sigma_Theta, double sigma_Accelerate, double sigma_AngleVelocity,ros::NodeHandle nh)
{
    this->param = param;
    this->map = map;
    this->x_center = x_center;
    this->y_center = y_center;

    this->sigma_X = sigma_X;
    this->sigma_Y = sigma_Y;
    this->sigma_Theta = sigma_Theta;
    this->sigma_Accelerate = sigma_Accelerate;
    this->sigma_AngleVelocity = sigma_AngleVelocity;

    this->uncertainty_cost.setZero();
    uncertainty_cost(0,0) = this->param.w_uncertainty;
    uncertainty_cost(1,1) = this->param.w_uncertainty;

    this->PX.setZero();
    PX(0)=1;
    this->PY.setZero();
    PY(1)=1;

    //grid map
    bool convert_flag=false;
    convert_flag=grid_map::GridMapRosConverter::fromMessage(grid_map,this->gridmap_uncertaintymap);
    // ROS_INFO("Grid Map msg convert!");
    // ROS_INFO("Covert successfull(1/0) %d",convert_flag);

    InitialUncertaintyMap();
    this->vehicle_length_half = this->param.length/2.0 + this->param.safe_length;//1.0
    this->vehicle_width_half = this->param.width/2.0 + this->param.safe_width;//1.0
    // plan 1 length:1.0 width 1.0 can pass 1234 both +0.5
    // plan 2 length:0 width 0.5 can pass 124 both +0.5
    // plan 3 length:0 width 0 can not pass both +0.5

    // scenario 1: 
    //      sigma 1: 
    //           length : 0.5 width:0.5 time 0.125 smooth
    //           length : 0.3 width:0.3 time 0.117 smooth
    //           length : 0.1 width:0.2 time 0.117 smooth
    //                      0.0     0.1 not

    this->nh = nh;
    this->elipse_pub = nh.advertise<visualization_msgs::MarkerArray>("Elipse", 10);
}
    



void Uncertainty::InitialUncertaintyMap()
{
    // transform Quaternion to roll, pitch, yaw
    tf::Quaternion quat;
    tf::quaternionMsgToTF(this->map.info.origin.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    if (yaw < 0)
    {
        yaw += 6.28318530718;
    }

    this->vehiclemap.x_center = x_center;
    this->vehiclemap.y_center = y_center;

    this->vehiclemap.Vx_og = map.info.origin.position.x;
    this->vehiclemap.Vy_og = map.info.origin.position.y;
    this->vehiclemap.width = map.info.width;
    this->vehiclemap.height = map.info.height;
    this->vehiclemap.heading = yaw;
    // std::cout<<"Map yaw"<<yaw<<std::endl;
    // ROS_INFO("Vehicle Map Pos x:%f,pos y:%f,heading :%f",map.info.origin.position.x,map.info.origin.position.y,yaw);
    // ROS_INFO("Center x:%f,Center y:%f",x_center,y_center);
    // for (auto iterator = map.data.rbegin();
    //      iterator != map.data.rend(); ++iterator)
    // {
    //     size_t i = std::distance(map.data.rbegin(), iterator);
    //     this->vehiclemap.data[(map.info.width - 1) - i % map.info.width][i / map.info.width] = *iterator;
    //     // ROS_INFO("%d %d %d", map.info.width-1-i%map.info.width, i/map.info.width, *iterator);
    //     // ros::Duration(0.01).sleep();
    // }
    Eigen::Matrix2f coviranceMatrix = this->getConfidenceMatrix();
    this->elipseParam_const = this->getConfidenceEllipse(coviranceMatrix,5.991);
}

/*
input:q1,q2,c,c_dot:2*1
output:
x:q1*e^(q2*c)
vx:[x,x].T 2*1
mx:[x,x;x,x;] 2*2
*/
struct_x_vx_mx_vmx Uncertainty::barrier_function(double q1, double q2, double c, Eigen::Vector4d c_dot)
{
    struct_x_vx_mx_vmx result;
    result.x = q1 * std::exp(q2 * c);
    result.vx = q2 * q1 * std::exp(q2 * c) * c_dot;

    Eigen::MatrixXd c_dot_m = c_dot;
    Eigen::MatrixXd c_dot_m_t = c_dot.transpose();

    result.mx = q2 * q2 * q1 * std::exp(q2 * c) * c_dot_m * c_dot_m_t;//Problem
    return result;
}
/*
input:ego_state x,y,v,yaw in global position
output: index in vehicle pos;
*/
Eigen::Vector2d Uncertainty::GlobalIndexInVehicleMap(Eigen::VectorXd ego_state)
{
    Eigen::Vector2d index;
    index<<-1,-1;
    double sin_Vtheta_og = sin(this->vehiclemap.heading);
    double cos_Vtheta_og = cos(this->vehiclemap.heading);

    double dx, dy; // difference of x,y between frenet and vechicle origin in global frame
    dx = ego_state(0) - this->vehiclemap.Vx_og;
    dy = ego_state(1) - this->vehiclemap.Vy_og;

    double Cx, Cy; // coordination of x,y in vehicle frame
    Cx = dx * cos_Vtheta_og + dy * sin_Vtheta_og;
    Cy = dx * sin_Vtheta_og - dy * cos_Vtheta_og;

    int index_x, index_y; // index of planned points in vehicle frame
    index_x = (int)(Cx / this->vehiclemap.resolution);
    index_y = (Cy + this->vehiclemap.y_center) / this->vehiclemap.resolution + this->vehiclemap.height / 2;
    if (Cx >= 0 && Cx <= this->vehiclemap.width && Cy >= (this->vehiclemap.y_center / this->vehiclemap.resolution - this->vehiclemap.height / 2) && Cy <= (this->vehiclemap.y_center / this->vehiclemap.resolution + this->vehiclemap.height / 2))
    {
        index<<index_x,index_y;
        return index;
    }
    return index;
}

bool Uncertainty::GlobalInVehicleMap(Eigen::VectorXd ego_state)
{
    double sin_Vtheta_og = sin(this->vehiclemap.heading);
    double cos_Vtheta_og = cos(this->vehiclemap.heading);

    double dx, dy; // difference of x,y between frenet and vechicle origin in global frame
    dx = ego_state(0) - this->vehiclemap.Vx_og;
    dy = ego_state(1) - this->vehiclemap.Vy_og;
    // ROS_INFO("x: %f, y: %f ,map_x: %f, map_y: %f", ego_state(0),ego_state(1),this->vehiclemap.Vx_og,this->vehiclemap.Vy_og);
    // ROS_INFO("dx: %f, dy: %f", dx, dy);
    double Cx, Cy; // coordination of x,y in vehicle frame
    Cx = dx * cos_Vtheta_og + dy * sin_Vtheta_og;
    Cy = dx * sin_Vtheta_og - dy * cos_Vtheta_og;

    int index_x, index_y; // index of planned points in vehicle frame
    index_x = (int)(Cx / this->vehiclemap.resolution);
    index_y = (Cy + this->vehiclemap.y_center) / this->vehiclemap.resolution + this->vehiclemap.height / 2;
    
    if (Cx >= 0 && Cx <= vehiclemap.width && Cy >= (vehiclemap.y_center / vehiclemap.resolution - vehiclemap.height / 2)
    && Cy <= (vehiclemap.y_center / vehiclemap.resolution  + vehiclemap.height / 2))
    {
        // ROS_INFO("In Cx:%d,Cy:%d,Global pose x:%f,global pose y:%f",index_x,index_y,ego_state(0),ego_state(1));
        return true;
    }
    // ROS_INFO("Out Cx:%d,Cy:%d,Global pose x:%f,global pose y:%f",index_x,index_y,ego_state(0),ego_state(1));
    return false;
}
/*
input:
output:covirance matrix
TODO:Control sigma.
*/
Eigen::Matrix2f Uncertainty::getConfidenceMatrix()
{
    double X = this->sigma_X;
    double Y = this->sigma_Y;

    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    cov(0, 0) = X * X;
    cov(1, 1) = Y * Y;
    return cov;
}
/*
input:covirance matrix, degree
output:param: major_axis,minor_axis,angle
*/
std::vector<double> Uncertainty::getConfidenceEllipse(Eigen::Matrix2f cov, double chisquare_val = 5.991)
{
    std::vector<double> params;

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

struct_x_vx_mx_vmx Uncertainty::get_uncertainty_cost(Eigen::VectorXd ego_state)
{
    struct_x_vx_mx_vmx result;
    result.x = 0;
    result.vx = Eigen::Vector4d::Zero();
    result.mx = Eigen::Matrix4d::Zero();
    
    // std::vector<std::pair<double,double>> elipse_points;

    
    // ROS_INFO("Get Uncertainty Constraint");
    if (GlobalInVehicleMap(ego_state))
    {
        std::vector<double> elipseParam = elipseParam_const;
        // Diag
        Eigen::MatrixX4d P1 = Eigen::Matrix4d::Zero();
        P1(0,0) = 1.0/( this->vehicle_width_half ) /( this->vehicle_width_half ) ;
        P1(1,1) = 1.0/( this->vehicle_length_half ) /( this->vehicle_length_half );
        // P1(0,0) = 1.0/(elipseParam[0] + this->param.width/2.0 + 1.0) /(elipseParam[0]+ this->param.width/2.0 +1.0) ;
        // P1(1,1) = 1.0/(elipseParam[1] + this->param.length/2.0 + 1.0) /(elipseParam[1]+ this->param.length/2.0 +1.0);
        // ROS_INFO("In uncertainty Map");
        // Eigen::Matrix2f coviranceMatrix = this->getConfidenceMatrix();
        // this->elipseParam_const = this->getConfidenceEllipse(coviranceMatrix);

        // ROS_INFO("a:%f,b:%f,theta:%f",elipseParam[0],elipseParam[1],elipseParam[2]);
        
        elipseParam[0] = elipseParam[0] + this->vehicle_width_half ;
        elipseParam[1] = elipseParam[1] + this->vehicle_length_half ; //
        // std::cout<<"changed"<<std::endl;
        // elipseParam[2] = elipseParam[2] - ego_state(3);
        // std::cout<<"elipse param 2 " <<elipseParam[2]<<" ego state theta "<< ego_state(3)<<" map theta "<<vehiclemap.heading<<std::endl;
        // elipseParam[2] = elipseParam[2] - ego_state(3) +this->vehiclemap.heading;
        // elipseParam[2] = elipseParam[2] -this->vehiclemap.heading; // in global theta;
        elipseParam[2] = elipseParam[2] -this->vehiclemap.heading + ego_state(3);
        

        // position for elipseIterator
        grid_map::Position position_vehicle;
        double sin_Vtheta_og = sin(this->vehiclemap.heading);
        double cos_Vtheta_og = cos(this->vehiclemap.heading);
        position_vehicle.x() = (ego_state(0) - this->vehiclemap.Vx_og) * cos_Vtheta_og + (ego_state(1) - this->vehiclemap.Vy_og) * sin_Vtheta_og;
        position_vehicle.y() = (ego_state(0) - this->vehiclemap.Vx_og) * (-sin_Vtheta_og) + (ego_state(1) - this->vehiclemap.Vy_og) * cos_Vtheta_og;

        // Each elipse c c_dot
        double c_elipse = 0;
        Eigen::Vector2d c_dot_elipse = Eigen::Vector2d::Zero();

        int elipse_cell_count = 0;
        for (grid_map::EllipseIterator iterator(this->gridmap_uncertaintymap, position_vehicle,grid_map::Length(2 * elipseParam[0], 2 * elipseParam[1]), elipseParam[2]);
             !iterator.isPastEnd(); ++iterator)
        {
            for (int i = 0; i < 4 && !iterator.isPastEnd(); ++i) {
                     ++iterator;
                }
            if(iterator.isPastEnd()) break;
            // ROS_INFO("Elipse Iterator Start");
            grid_map::Position position_vehicle_it;
            this->gridmap_uncertaintymap.getPosition(*iterator, position_vehicle_it);
            
            // Uncertainty value 
            static double uncertainty_value;
            uncertainty_value = this->gridmap_uncertaintymap.atPosition("uncertainty_map", position_vehicle_it) /100.0;//P 0-100 /100 = probality.

            // position in global
            static double global_x,global_y;
            global_x = position_vehicle_it.x() * cos_Vtheta_og - position_vehicle_it.y() * sin_Vtheta_og + vehiclemap.Vx_og;
            global_y = position_vehicle_it.x() * sin_Vtheta_og + position_vehicle_it.y() * cos_Vtheta_og + vehiclemap.Vy_og;
            // elipse_points.push_back({global_x,global_y});

            if(uncertainty_value < 0.001) continue;
            // std::cout<<uncertainty_value<<std::endl;
            // ROS_INFO("Value: %f Pos X: %f Pos Y: %f",uncertainty_value,global_x,global_y);

            // Relative distance
            Eigen::Vector4d temp=ego_state;
            temp(0) = temp(0) - global_x;
            temp(1) = temp(1) - global_y;

            // std::cout<<"Temp:"<<temp<<std::endl;

            Eigen::MatrixXd c_matrix = temp.transpose() * P1 * temp;
            Eigen::MatrixXd c_dot_matrix = -2 * P1 * temp;
            // std::cout<<"C_matrix"<<c_matrix<<std::endl;
            // std::cout<<"C_dot_matrix"<<c_dot_matrix<<std::endl;

            double c =1 - c_matrix(0);
            Eigen::Vector4d c_dot = c_dot_matrix;

            // std::cout<<"C: "<<c<<std::endl;
            // std::cout<<"C_dot: "<<c_dot<<std::endl;

            struct_x_vx_mx_vmx point_ans = this->barrier_function(this->param.q1_uncertainty,this->param.q2_uncertainty,c,c_dot);
            result.x = result.x + point_ans.x * uncertainty_value;
            result.vx = result.vx + point_ans.vx * uncertainty_value;
            result.mx = result.mx + point_ans.mx * uncertainty_value;
            // result.x = result.x + point_ans.x * std::exp(uncertainty_value);
            // result.vx = result.vx + point_ans.vx * std::exp(uncertainty_value);
            // result.mx = result.mx + point_ans.mx * std::exp(uncertainty_value);

            elipse_cell_count++;
            // ++iterator;
            // ++iterator;
            // ++iterator;
            // ++iterator;

        }
        // this->ElipseVisualization(elipse_points);
        // if(elipse_cell_count >0)
        // {
        //     result.x = result.x /1.0/ elipse_cell_count;
        //     result.vx = result.vx /1.0/ elipse_cell_count;
        //     result.mx = result.mx /1.0/ elipse_cell_count;
        // }

        return result;
        // ROS_INFO("Map resolution :%fm",this->vehiclemap.resolution);
        // ROS_INFO("Uncertainty Sigma X:%f m, Uncertainty Sigma Y :%f m",this->sigma_X,this->sigma_Y);
        // ROS_INFO("Elipse Length :%f m, Elipse Width :%f m",elipseParam[0],elipseParam[1]);
        // ROS_INFO("Elipse Cell Count is %d",elipse_cell_count);
        
        // X is heading Y is Left.
    }
    else
    {
        // ROS_INFO("Not in VehicleMap, relative position x:%f y:%f",ego_state(0),ego_state(1));
        return result;
    }
}
/*
Function:PathVisualization.
*/
void Uncertainty::ElipseVisualization(std::vector<std::pair<double,double>> &Elipse)
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
    marker.lifetime = ros::Duration(10.0); // 显示时间为1秒
    for (int i = 0; i < Elipse.size(); i++)
    {
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        marker.pose.position.x = Elipse[i].first;
        marker.pose.position.y = Elipse[i].second;
        marker.pose.position.z = 0.0;
        path_markerarray.markers.emplace_back(marker);
    }
    this->elipse_pub.publish(path_markerarray);
}

// 计算二维高斯分布的概率密度函数
double Uncertainty::gaussian_fx(double x, double y, const GaussianParams& params) {
    double norm_factor = 1.0 / (2 * M_PI * params.sigma_x * params.sigma_y * std::sqrt(1 - params.rho_pow2));
    double exp_factor = std::exp(-1.0 / (2 * (1 - params.rho_pow2)) * (
        std::pow((x - params.mu_x) / params.sigma_x, 2) +
        std::pow((y - params.mu_y) / params.sigma_y, 2) -
        2 * params.rho * (x - params.mu_x) * (y - params.mu_y) / (params.sigma_x * params.sigma_y)
    ));
    ROS_INFO("x: %f,y:%f",x,y);
    ROS_INFO("U_x: %f,U_y:%f",params.mu_x,params.mu_y);
    ROS_INFO("rho: %f",params.rho);
    return norm_factor * exp_factor;
}

void Uncertainty::gaussian_fx_derivative(double x, double y, const GaussianParams& params,double &fx,Eigen::Vector2d &fx_d) {
    double fx_value = gaussian_fx(x, y, params);

    double derivative_x = fx_value * (-1.0 / (1 - params.rho_pow2)) * (
        (x - params.mu_x) / (params.sigma_x_pow2) -
        params.rho * (y - params.mu_y) / (params.sigma_x_multiple_sigma_y)
    );
    double derivative_y = fx_value * (-1.0 / (1 - params.rho_pow2)) * (
        (y - params.mu_y) / (params.sigma_y * params.sigma_y) -
        params.rho * (x - params.mu_x) / (params.sigma_x_multiple_sigma_y)
    );

    fx = fx_value;
    fx_d << derivative_x,derivative_y;
    ROS_INFO("x: %f,y:%f",x,y);
    ROS_INFO("C:%f,C_dot:%f,%f",fx,fx_d(0),fx_d(1));

}

void Uncertainty::createRotatedRectangle(grid_map::Polygon& polygon, const grid_map::Position& center, double length, double width, double angle) {
    // Define the four corners of the rectangle
    Eigen::Vector2d half_length_vector = 0.5 * length * Eigen::Vector2d(cos(angle), sin(angle));
    Eigen::Vector2d half_width_vector = 0.5 * width * Eigen::Vector2d(-sin(angle), cos(angle));

    polygon.addVertex(center + half_length_vector + half_width_vector);
    polygon.addVertex(center + half_length_vector - half_width_vector);
    polygon.addVertex(center - half_length_vector - half_width_vector);
    polygon.addVertex(center - half_length_vector + half_width_vector);
}
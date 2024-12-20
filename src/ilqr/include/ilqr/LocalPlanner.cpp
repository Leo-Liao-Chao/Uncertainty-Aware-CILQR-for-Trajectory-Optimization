#include "LocalPlanner.h"

LocalPlanner::LocalPlanner(const Parameters &param)
{
    this->param = param;
}
/*
input:ego_state: 4*1 [x,y,v,theta].T
*/
void LocalPlanner::set_ego_state(const Eigen::Vector4d &ego_state)
{
    this->ego_state = ego_state;
}
/*
input:global_plan_points 2*n
*/
void LocalPlanner::set_global_planner(const Eigen::MatrixXd &global_plan_points)
{
    this->global_plan_points = global_plan_points;
}
/*
input:point:4*1 [x,y,v,theta].T
output:closest point:2*1
*/
int LocalPlanner::closest_point_index(const Eigen::Vector4d &point)
{
    double min_distance = std::pow(point(0) - global_plan_points(0, 0), 2) + std::pow(point(1) - global_plan_points(1, 0), 2);
    int min_index = 0;
    double temp_distance;

    for (int i = 0; i < global_plan_points.cols(); i++)
    {
        temp_distance =std::pow(point(0) - global_plan_points(0, i), 2) + std::pow(point(1) - global_plan_points(1, i), 2);
        if (temp_distance < min_distance)
        {
            min_distance = temp_distance;
            min_index = i;
        }
    }
    return min_index;
}
/*
output: points 2*n
function:find the closest point of pos_now in global points ,
         and return the closest point and the later in global points;
*/
Eigen::MatrixXd LocalPlanner::get_local_wpts()
{
    int points_index = this->closest_point_index(this->ego_state);
    double nums_local_wpts;
    if((this->global_plan_points.cols() - points_index) < this->param.num_of_local_wpts)
    {
        nums_local_wpts = this->global_plan_points.cols() - points_index;
    }
    else
    {
        nums_local_wpts = this->param.num_of_local_wpts;
    }
    return this->global_plan_points.block(0, points_index, this->global_plan_points.rows(), nums_local_wpts);
}
/*
output: local_plan_refer_traj:2*n
function:Based on the pos_now , find the next path in global path;
         And make use of polyfit to fit local_plan_refer_traj by the next path;
*/
Eigen::MatrixXd LocalPlanner::get_local_plan()
{
    Eigen::MatrixXd local_wpts = this->get_local_wpts(); // local_points in global plan
    Eigen::VectorXd coeffs = this->polyfit(local_wpts.row(0), local_wpts.row(1), this->param.poly_order);

    Eigen::VectorXd new_y = Eigen::VectorXd::Zero(local_wpts.cols());

    for (int i = 0; i < local_wpts.cols(); i++)
    {
        for (int j = 0; j < this->param.poly_order + 1; j++)
        {
            new_y(i) += coeffs(j) * std::pow(local_wpts(0,i), j);
        }
    }

    Eigen::MatrixXd local_plan(local_wpts.rows(), local_wpts.cols());
    local_plan.row(0) = local_wpts.row(0);
    local_plan.row(1) = new_y.transpose();
    return local_plan;
}
/*
output:coeffs n*1 n = ploy_order
function: return the coeffs of local_plan_refer_traj;
*/
Eigen::VectorXd LocalPlanner::get_local_plan_coeffs()
{
    Eigen::MatrixXd local_wpts = this->get_local_wpts(); // local_points in global plan
    Eigen::VectorXd coeffs = this->polyfit(local_wpts.row(0), local_wpts.row(1), this->param.poly_order);

    return coeffs;
}
/*
input: x:n*1 y:n*1 degree:ploy_order
output:coeffs:(ploy_order+1)*1
*/
Eigen::VectorXd LocalPlanner::polyfit(const Eigen::VectorXd &x, const Eigen::VectorXd &y, int &degree)
{
    int n = x.rows();

    Eigen::MatrixXd X =Eigen::MatrixXd::Zero(n, degree + 1);
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j <= degree; ++j)
        {
            X(i, j) = std::pow(x(i), j);
        }
    }
    // 使用最小二乘法求解系数
    Eigen::VectorXd coeffs = X.colPivHouseholderQr().solve(y);

    return coeffs;
}
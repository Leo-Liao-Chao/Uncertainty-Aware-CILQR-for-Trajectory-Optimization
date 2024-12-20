#include "Model.h"

Model::Model(const Parameters &params)
{
    this->params = params;
}
/*
input:state=[x,y,v,theta].T control=[acc,yaw_acc].T
output:next_state=[x,y,v,theta].T
function:
First  , let control in limit
Second , x=x+cos(theta)(v*t+0.5a*t*t),
         y=sin(theta)(v*t+0.5a*t*t)  ,
         v=v+at(speed in limit)      ,
         theta=theta +a_theta*t
*/
Eigen::Vector4d Model::forward_simulate(const Eigen::Vector4d &state, Eigen::Vector2d control)
{
    control(0) = std::max(std::min(control(0), this->params.acc_max), this->params.acc_min);
    control(1) = std::max(std::min(control(1), state[2]*std::tan(this->params.steer_angle_max)/this->params.wheelbase), state[2]*std::tan(this->params.steer_angle_min)/this->params.wheelbase);

    Eigen::Vector4d next_state;

    next_state(0) = state(0) + std::cos(state(3)) * (state(2) * this->params.timestep + control(0) * this->params.timestep * this->params.timestep / 2.0);
    next_state(1) = state(1) + std::sin(state(3)) * (state(2) * this->params.timestep + control(0) * this->params.timestep * this->params.timestep / 2.0);
    next_state(2) = std::min(std::max(state(2) + control(0) * this->params.timestep, 0.0), this->params.speed_max);
    next_state(3) = state(3) + control(1) * this->params.timestep;

    return next_state;
}
// /*
// input: v:n*1 theta:n*1 acc:n*1
// output:
// A：4x4*horizon
// [
//     1, 0 ,cos(theta)*Ts , -sin(theta)(v*Ts+1/2(a*Ts*Ts)
//     0, 1 ,sin(theta)*Ts , cos(theta)(v*Ts+1/2(a*Ts*Ts)
//     0, 0 ,1             , 0
//     0, 0 ,0             , 1
// ].T
// */
// std::vector<Eigen::MatrixXd> Model::vector_get_A_matrix(const Eigen::VectorXd &velocity_vals, const Eigen::VectorXd &theta, const Eigen::VectorXd &acceleration_vals)
// {
//     std::vector<Eigen::MatrixXd> A;
//     A.reserve(this->params.horizon);
//     Eigen::MatrixXd matrix_temp(4, 4);

//     for (int i = 0; i < this->params.horizon; i++)
//     {
//         matrix_temp.col(0) << 1.0, 0.0, this->params.timestep * std::cos(theta(i)), (-1)*std::sin(theta(i)) * (velocity_vals(i) * this->params.timestep + 0.5 * acceleration_vals(i) * this->params.timestep * this->params.timestep);
//         matrix_temp.col(1) << 0.0, 1.0, this->params.timestep * std::sin(theta(i)), std::cos(theta(i)) * (velocity_vals(i) * this->params.timestep + 0.5 * acceleration_vals(i) * this->params.timestep * this->params.timestep);
//         matrix_temp.col(2) << 0.0, 0.0, 1.0, 0.0;
//         matrix_temp.col(3) << 0.0, 0.0, 0.0, 1.0;

//         A.push_back(matrix_temp);
//     }
//     return A;
// }
// /*
// input: theta:n*1
// output:
// B：2x4*horizon
// [
//     cos(theta)(dt)(dt)/2, 0
//     sin(theta)(dt)(dt)/2, 0
//     timestep            , 0
//     0                   , timestep
// ].T
// */
// std::vector<Eigen::MatrixXd> Model::vector_get_B_matrix(const Eigen::VectorXd &theta)
// {
//     std::vector<Eigen::MatrixXd> B;

//     B.reserve(this->params.horizon);
//     Eigen::MatrixXd matrix_temp(2, 4);

//     for (int i = 0; i < this->params.horizon; i++)
//     {
//         matrix_temp.col(0) << this->params.timestep * this->params.timestep * std::cos(theta(i)) / 2.0, 0.0;
//         matrix_temp.col(1) << this->params.timestep * this->params.timestep * std::sin(theta(i)) / 2.0, 0.0;
//         matrix_temp.col(2) << this->params.timestep, 0.0;
//         matrix_temp.col(3) << 0.0, this->params.timestep;

//         B.push_back(matrix_temp);
//     }
//     return B;
// }

/*
input: v:n*1 theta:n*1 acc:n*1
output:
A：4x4*horizon
[
    1, 0 ,cos(theta)*Ts , -sin(theta)(v*Ts+1/2(a*Ts*Ts)
    0, 1 ,sin(theta)*Ts , cos(theta)(v*Ts+1/2(a*Ts*Ts)
    0, 0 ,1             , 0
    0, 0 ,0             , 1
].T
*/
Eigen::Tensor<double, 3> Model::get_A_matrix(const Eigen::VectorXd &velocity_vals, const Eigen::VectorXd &theta, const Eigen::VectorXd &acceleration_vals)
{
    Eigen::Tensor<double, 3> A(4, 4, this->params.horizon);

    for (int i = 0; i < this->params.horizon; i++)
    {
        A(0, 0, i) = 1.0;
        A(1, 0, i) = 0.0;
        A(2, 0, i) = this->params.timestep * std::cos(theta(i));
        A(3, 0, i) = (-1) * std::sin(theta(i)) * (velocity_vals(i) * this->params.timestep + 0.5 * acceleration_vals(i) * this->params.timestep * this->params.timestep);
        
        A(0, 1, i) = 0.0;
        A(1, 1, i) = 1.0;
        A(2, 1, i) = this->params.timestep * std::sin(theta(i));
        A(3, 1, i) = std::cos(theta(i)) * (velocity_vals(i) * this->params.timestep + 0.5 * acceleration_vals(i) * this->params.timestep * this->params.timestep);
        
        A(0, 2, i) = 0.0;
        A(1, 2, i) = 0.0;
        A(2, 2, i) = 1.0;
        A(3, 2, i) = 0.0;
        
        A(0, 3, i) = 0.0;
        A(1, 3, i) = 0.0;
        A(2, 3, i) = 0.0;
        A(3, 3, i) = 1.0;
    }
    return A;
}
/*
input: theta:n*1
output:
B：2x4*horizon
[
    cos(theta)(dt)(dt)/2, 0
    sin(theta)(dt)(dt)/2, 0
    timestep            , 0
    0                   , timestep
].T
*/
Eigen::Tensor<double, 3> Model::get_B_matrix(const Eigen::VectorXd &theta)
{
    Eigen::Tensor<double, 3> B(2, 4, this->params.horizon);

    for (int i = 0; i < this->params.horizon; i++)
    {
        B(0, 0, i) = this->params.timestep * this->params.timestep * std::cos(theta(i)) / 2.0;
        B(1, 0, i) = 0.0;
        B(0, 1, i) = this->params.timestep * this->params.timestep * std::sin(theta(i)) / 2.0;
        B(1, 1, i) = 0.0;
        B(0, 2, i) = this->params.timestep;
        B(1, 2, i) = 0.0;
        B(0, 3, i) = 0.0;
        B(1, 3, i) = this->params.timestep;
    }
    return B;
}
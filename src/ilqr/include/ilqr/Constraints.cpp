#include "Constraints.h"

Constraints::Constraints(const Parameters &param)
{
    this->control_cost.setZero();
    control_cost(0, 0) = param.w_acc;
    control_cost(1, 1) = param.w_yawrate;

    this->state_cost.setZero();
    state_cost(0, 0) = param.w_pos;
    state_cost(1, 1) = param.w_pos;
    state_cost(2, 2) = param.w_vel;

    this->param = param;
}

/*
input:state:4*1=[x,y,v,yaw].T,coeffs:6*1=[a,b,c,d,e,f].T ploy 5,x_local_plan:n*1=[x,x,x,x,x,x,x...].T
output:[x,y].T
base coeffs and x_local_plan[0],[-1], get new path;
find the closest point, and its index c;
return point [x,y].T
*/
Eigen::Vector2d Constraints::find_closest_point(const Eigen::Vector4d &state, const Eigen::VectorXd &coeffs, const Eigen::VectorXd &x_local_plan)
{
    Eigen::Vector2d closest_point;

    Eigen::VectorXd new_x = Eigen::VectorXd::Zero(this->param.num_of_local_wpts * 10);
    Eigen::VectorXd new_y = Eigen::VectorXd::Zero(this->param.num_of_local_wpts * 10);

    int x_local_plan_length = x_local_plan.size(); // 长度
    double dx = (x_local_plan(x_local_plan_length - 1) - x_local_plan(0)) / (this->param.num_of_local_wpts * 10);
    double start_x = x_local_plan(0);

    for (int i = 0; i < param.num_of_local_wpts * 10; i++)
    {
        new_x(i) = start_x + dx * i;
        for (int j = 0; j < this->param.poly_order + 1; j++)
        {
            new_y(i) += coeffs(j) * std::pow(new_x(i), j);
        }
    }
    double min_distance = (new_x(0) - state(0)) * (new_x(0) - state(0)) + (new_y(0) - state(1)) * (new_y(0) - state(1)); // First point distance**2
    double temp_distance = 0;
    int min_index = 0;

    for (int i = 0; i < param.num_of_local_wpts * 10; i++)
    {
        temp_distance = (new_x(i) - state(0)) * (new_x(i) - state(0)) + (new_y(i) - state(1)) * (new_y(i) - state(1));
        if (temp_distance < min_distance)
        {
            min_distance = temp_distance;
            min_index = i;
        }
    }
    closest_point << new_x(min_index), new_y(min_index);

    return closest_point;
}
/*
input:q1,q2,c,c_dot:2*1
output:
x:q1*e^(q2*c)
vx:[x,x].T 2*1
mx:[x,x;x,x;] 2*2
*/
struct_x_vx_mx_vmx Constraints::barrier_function(const double &q1, const double &q2, const double &c, const Eigen::Vector2d &c_dot)
{
    struct_x_vx_mx_vmx result;
    result.x = q1 * std::exp(q2 * c);
    result.vx = q2 * q1 * std::exp(q2 * c) * c_dot;

    Eigen::Matrix<double, 2, 1> c_dot_m = c_dot;
    Eigen::Matrix<double, 1, 2> c_dot_m_t = c_dot.transpose();

    result.mx = q2 * q2 * q1 * std::exp(q2 * c) * c_dot_m * c_dot_m_t;
    return result;
}
/*
input:state:4*n,control:2*n
output:
mx:l_u:2*n  dx
vmx:l_uu:2*2*n  dxx
function:compute control cost l_u l_uu
*/
struct_x_vx_mx_vmx Constraints::get_control_cost(const Eigen::MatrixXd &state, const Eigen::MatrixXd &control)
{
    struct_x_vx_mx_vmx control_cost;

    Eigen::Vector2d P1;
    P1.row(0) << 1;
    P1.row(1) << 0;
    Eigen::Vector2d P2;
    P2.row(0) << 0;
    P2.row(1) << 1;

    Eigen::MatrixXd l_u = Eigen::MatrixXd::Zero(this->param.num_ctrls, this->param.horizon);
    // std::vector<Eigen::MatrixXd> l_uu;
    // l_uu.reserve(this->param.horizon);
    Eigen::Tensor<double, 3> l_uu(this->param.num_ctrls, this->param.num_ctrls, this->param.horizon);
    l_uu.setZero();

    Eigen::MatrixXd temp_matrix;
    Eigen::MatrixXd c_dot_dot_i = Eigen::MatrixXd::Zero(this->param.num_ctrls, this->param.num_ctrls);

    double c = 0;

    struct_x_vx_mx_vmx cost_1, cost_2, cost_3, cost_4;

    for (int i = 0; i < this->param.horizon; i++)
    {
        temp_matrix = control.col(i).transpose() * P1;
        c = temp_matrix(0, 0) - this->param.acc_max;
        cost_1 = this->barrier_function(this->param.q1_acc, this->param.q2_acc, c, P1);
        c = this->param.acc_min - temp_matrix(0, 0);
        cost_2 = this->barrier_function(this->param.q1_acc, this->param.q2_acc, c, -P1);

        temp_matrix = control.col(i).transpose() * P2;
        c = temp_matrix(0, 0) - state(2, i) * std::tan(this->param.steer_angle_max) / this->param.wheelbase;
        cost_3 = this->barrier_function(this->param.q1_yawrate, this->param.q2_yawrate, c, P2);
        c = state(2, i) * std::tan(this->param.steer_angle_min) / this->param.wheelbase - temp_matrix(0, 0);

        cost_4 = this->barrier_function(this->param.q1_yawrate, this->param.q2_yawrate, c, -P2);

        l_u.col(i) << cost_1.vx + cost_2.vx + cost_3.vx + cost_4.vx + 2 * this->control_cost * control.col(i);
        c_dot_dot_i = cost_1.mx + cost_2.mx + cost_3.mx + cost_4.mx + 2 * this->control_cost;

        l_uu(0, 0, i) = c_dot_dot_i(0, 0);
        l_uu(0, 1, i) = c_dot_dot_i(0, 1);
        l_uu(1, 0, i) = c_dot_dot_i(1, 0);
        l_uu(1, 1, i) = c_dot_dot_i(1, 1);
    }
    control_cost.mx = l_u;
    control_cost.tx = l_uu;

    return control_cost;
}
/*
input:state:4*n,poly_coeffs:n*1 n=ploy,x_local_plan:n*1;
output:
mx:l_x:4*n
vmx:l_xx:4*4*n
function:compute state cost l_x l_xx
*/
struct_x_vx_mx_vmx Constraints::get_state_cost(const Eigen::MatrixXd &state, const Eigen::VectorXd &poly_coeffs, const Eigen::VectorXd &x_local_plan)
{
    struct_x_vx_mx_vmx result;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
    Eigen::MatrixXd l_x = Eigen::MatrixXd::Zero(this->param.num_states, this->param.horizon);
    Eigen::Tensor<double, 3> l_xx(this->param.num_states, this->param.num_states, this->param.horizon);
    l_xx.setZero();

    Eigen::Vector2d closest_point_temp;
    Eigen::Matrix<double, 4, 1> temp;

    Eigen::Vector4d traj_cost;
    Eigen::Vector4d l_x_i = Eigen::Vector4d::Zero(this->param.num_states);

    Eigen::Matrix4d l_xx_i = Eigen::Matrix4d::Zero(this->param.num_states, this->param.num_states);

    for (int i = 0; i < this->param.horizon; i++)
    {
        closest_point_temp = this->find_closest_point(state.col(i), poly_coeffs, x_local_plan); // N*2

        temp.row(0) << state(0, i) - closest_point_temp(0);     // x
        temp.row(1) << state(1, i) - closest_point_temp(1);     // y
        temp.row(2) << state(2, i) - this->param.desired_speed; // v
        temp.row(3) << 0;

        traj_cost = 2 * this->state_cost * temp; // 4*4 4*1

        l_x_i = traj_cost;

        l_xx_i = this->state_cost * 2;

        struct_x_vx_mx_vmx obstacle_temple;
        if (this->obstacles.size() != 0)
        {
            // std::cout<<"Obstacle size %d"<<obstacles.size()<<std::endl;
            for (int j = 0; j < this->obstacles.size(); j++)
            {
                obstacle_temple = this->obstacles[j].get_obstalce_cost(i, state.col(i));

                l_x_i = l_x_i + obstacle_temple.vx * this->param.w_obstacle;
                l_xx_i = l_xx_i + obstacle_temple.mx * this->param.w_obstacle;
            }
        }
        if (this->flag_uncertainty_map_use == true)
        {
            struct_x_vx_mx_vmx uncertianty_temp;

            //  std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
            uncertianty_temp = this->uncertainty_map.get_uncertainty_cost(state.col(i));

            // std::chrono::time_point<std::chrono::high_resolution_clock> current_time = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double> elapsed_seconds = current_time - start_time; 
            // std::cout << "Time is " << elapsed_seconds.count() << " seconds" << std::endl;

            l_x_i += uncertianty_temp.vx * this->param.w_uncertainty;
            l_xx_i += uncertianty_temp.mx * this->param.w_uncertainty;
        }
        l_x.col(i) = l_x_i;

        l_xx(0, 0, i) = l_xx_i(0, 0);
        l_xx(0, 1, i) = l_xx_i(0, 1);
        l_xx(0, 2, i) = l_xx_i(0, 2);
        l_xx(0, 3, i) = l_xx_i(0, 3);
        l_xx(1, 0, i) = l_xx_i(1, 0);
        l_xx(1, 1, i) = l_xx_i(1, 1);
        l_xx(1, 2, i) = l_xx_i(1, 2);
        l_xx(1, 3, i) = l_xx_i(1, 3);
        l_xx(2, 0, i) = l_xx_i(2, 0);
        l_xx(2, 1, i) = l_xx_i(2, 1);
        l_xx(2, 2, i) = l_xx_i(2, 2);
        l_xx(2, 3, i) = l_xx_i(2, 3);
        l_xx(3, 0, i) = l_xx_i(3, 0);
        l_xx(3, 1, i) = l_xx_i(3, 1);
        l_xx(3, 2, i) = l_xx_i(3, 2);
        l_xx(3, 3, i) = l_xx_i(3, 3);
    }
    // std::chrono::time_point<std::chrono::high_resolution_clock> current_time = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> elapsed_seconds = current_time - start_time; 
    // std::cout << "Time is " << elapsed_seconds.count() << " seconds" << std::endl;
    result.mx = l_x;
    result.tx = l_xx;
    return result;
}

// IterationResult Constraints::process_iteration(int i, const Eigen::MatrixXd &state, const Eigen::VectorXd &poly_coeffs, 
//                        const Eigen::VectorXd &x_local_plan, const Constraints &constraints)
// {
//     // std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
//     Constraints non_const_constraints = constraints;
//     Eigen::Vector2d closest_point_temp = non_const_constraints.find_closest_point(state.col(i), poly_coeffs, x_local_plan);

//     // Eigen::Vector2d closest_point_temp = non_const_constraints.find_closest_point(state.col(i), poly_coeffs, x_local_plan);

//     Eigen::Matrix<double, 4, 1> temp;
//     temp.row(0) << state(0, i) - closest_point_temp(0);
//     temp.row(1) << state(1, i) - closest_point_temp(1);
//     temp.row(2) << state(2, i) - non_const_constraints.param.desired_speed;
//     temp.row(3) << 0;

//     Eigen::Vector4d traj_cost = 2 * non_const_constraints.state_cost * temp;
//     Eigen::Vector4d l_x_i = traj_cost;

//     Eigen::Matrix4d l_xx_i = non_const_constraints.state_cost * 2;

//     // std::chrono::time_point<std::chrono::high_resolution_clock> current_time = std::chrono::high_resolution_clock::now();
//     // std::chrono::duration<double> elapsed_seconds = current_time - start_time; 
//     // std::cout << "Copy  Time is " << elapsed_seconds.count() << " seconds" << std::endl;

//     // struct_x_vx_mx_vmx obstacle_temple;
//     // if (non_const_constraints.obstacles.size() != 0)
//     // {
//     //     for (int j = 0; j < non_const_constraints.obstacles.size(); j++)
//     //     {
//     //         obstacle_temple = non_const_constraints.obstacles[j].get_obstalce_cost(i, state.col(i));
//     //         l_x_i = l_x_i + obstacle_temple.vx * non_const_constraints.param.w_obstacle;
//     //         l_xx_i = l_xx_i + obstacle_temple.mx * non_const_constraints.param.w_obstacle;
//     //     }
//     // }
//     Uncertainty non_const_uncertainty = non_const_constraints.uncertainty_map;

//     // if (non_const_constraints.flag_uncertainty_map_use == true)
//     // {
//     //     struct_x_vx_mx_vmx uncertianty_temp = non_const_uncertainty.get_uncertainty_cost(state.col(i));
//     //     l_x_i += uncertianty_temp.vx;
//     //     l_xx_i += uncertianty_temp.mx;
//     // }
//     IterationResult iterationresult;
//     iterationresult.i = i;
//     iterationresult.l_x_i =l_x_i;
//     iterationresult.l_xx_i = l_xx_i;
//     return iterationresult;

//     // l_x.col(i) = l_x_i;

//     // l_xx(0, 0, i) = l_xx_i(0, 0);
//     // l_xx(0, 1, i) = l_xx_i(0, 1);
//     // l_xx(0, 2, i) = l_xx_i(0, 2);
//     // l_xx(0, 3, i) = l_xx_i(0, 3);
//     // l_xx(1, 0, i) = l_xx_i(1, 0);
//     // l_xx(1, 1, i) = l_xx_i(1, 1);
//     // l_xx(1, 2, i) = l_xx_i(1, 2);
//     // l_xx(1, 3, i) = l_xx_i(1, 3);
//     // l_xx(2, 0, i) = l_xx_i(2, 0);
//     // l_xx(2, 1, i) = l_xx_i(2, 1);
//     // l_xx(2, 2, i) = l_xx_i(2, 2);
//     // l_xx(2, 3, i) = l_xx_i(2, 3);
//     // l_xx(3, 0, i) = l_xx_i(3, 0);
//     // l_xx(3, 1, i) = l_xx_i(3, 1);
//     // l_xx(3, 2, i) = l_xx_i(3, 2);
//     // l_xx(3, 3, i) = l_xx_i(3, 3);
// }


// struct_x_vx_mx_vmx Constraints::get_state_cost(const Eigen::MatrixXd &state, const Eigen::VectorXd &poly_coeffs, const Eigen::VectorXd &x_local_plan)
// {
//     struct_x_vx_mx_vmx result;

//     Eigen::MatrixXd l_x = Eigen::MatrixXd::Zero(this->param.num_states, this->param.horizon);
//     Eigen::Tensor<double, 3> l_xx(this->param.num_states, this->param.num_states, this->param.horizon);
//     l_xx.setZero();



//     // int num_threads = 8; // 根据物理核心数设置线程数
//     // int batch_size = this->param.horizon / num_threads;
//     // std::vector<std::future<std::vector<IterationResult>>> futures;

//     // for (int t = 0; t < num_threads; ++t) {
//     //     futures.push_back(std::async(std::launch::async, [&, t] {
//     //         std::vector<IterationResult> results;
//     //         for (int i = t * batch_size; i < (t + 1) * batch_size; ++i) {
//     //             if(i>=this->param.horizon)continue;
//     //             IterationResult result = process_iteration(i, state, poly_coeffs, x_local_plan, *this);
//     //             results.push_back(result);
//     //         }
//     //         return results;
//     //     }));
//     // }
//     int num_threads = 2; // 根据物理核心数设置线程数
//     int batch_size = this->param.horizon / num_threads;
//     std::vector<std::future<std::vector<IterationResult>>> futures;

//     static ThreadPool pool(2);  // 创建一个包含8个线程的线程池

//     std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();

//     for (int t = 0; t < num_threads; ++t) {
//         futures.push_back(pool.enqueue([&, t] {
//             std::vector<IterationResult> results;
//             for (int i = t * batch_size; i < (t + 1) * batch_size; ++i) {
//                 if (i >= this->param.horizon) break;
//                 IterationResult result = process_iteration(i, state, poly_coeffs, x_local_plan, *this);
//                 // results.push_back(result);
//                 l_x.col(i) = result.l_x_i;
//                 l_xx(0, 0, i) = result.l_xx_i(0, 0);
//                 l_xx(0, 1, i) = result.l_xx_i(0, 1);
//                 l_xx(0, 2, i) = result.l_xx_i(0, 2);
//                 l_xx(0, 3, i) = result.l_xx_i(0, 3);
//                 l_xx(1, 0, i) = result.l_xx_i(1, 0);
//                 l_xx(1, 1, i) = result.l_xx_i(1, 1);
//                 l_xx(1, 2, i) = result.l_xx_i(1, 2);
//                 l_xx(1, 3, i) = result.l_xx_i(1, 3);
//                 l_xx(2, 0, i) = result.l_xx_i(2, 0);
//                 l_xx(2, 1, i) = result.l_xx_i(2, 1);
//                 l_xx(2, 2, i) = result.l_xx_i(2, 2);
//                 l_xx(2, 3, i) = result.l_xx_i(2, 3);
//                 l_xx(3, 0, i) = result.l_xx_i(3, 0);
//                 l_xx(3, 1, i) = result.l_xx_i(3, 1);
//                 l_xx(3, 2, i) = result.l_xx_i(3, 2);
//                 l_xx(3, 3, i) = result.l_xx_i(3, 3);  
//             }
//             return results;
//         }));
//     }



//     // std::cout<<"Level 1 %d"<<futures.size()<<std::endl;

//     // 等待所有线程完成
//     for (auto &future : futures) {
//         std::vector<IterationResult> results = future.get();
//         // 处理每个线程的结果
//         // for (const auto& result : results) {
//         //     // 处理result，例如合并到最终的l_x和l_xx中
//         //         int i = result.i;
//         //         // std::cout<<i<<std::endl;
//         //         l_x.col(i) = result.l_x_i;
//         //         l_xx(0, 0, i) = result.l_xx_i(0, 0);
//         //         l_xx(0, 1, i) = result.l_xx_i(0, 1);
//         //         l_xx(0, 2, i) = result.l_xx_i(0, 2);
//         //         l_xx(0, 3, i) = result.l_xx_i(0, 3);
//         //         l_xx(1, 0, i) = result.l_xx_i(1, 0);
//         //         l_xx(1, 1, i) = result.l_xx_i(1, 1);
//         //         l_xx(1, 2, i) = result.l_xx_i(1, 2);
//         //         l_xx(1, 3, i) = result.l_xx_i(1, 3);
//         //         l_xx(2, 0, i) = result.l_xx_i(2, 0);
//         //         l_xx(2, 1, i) = result.l_xx_i(2, 1);
//         //         l_xx(2, 2, i) = result.l_xx_i(2, 2);
//         //         l_xx(2, 3, i) = result.l_xx_i(2, 3);
//         //         l_xx(3, 0, i) = result.l_xx_i(3, 0);
//         //         l_xx(3, 1, i) = result.l_xx_i(3, 1);
//         //         l_xx(3, 2, i) = result.l_xx_i(3, 2);
//         //         l_xx(3, 3, i) = result.l_xx_i(3, 3);  
//         // }
//     }

//     std::chrono::time_point<std::chrono::high_resolution_clock> current_time = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> elapsed_seconds = current_time - start_time; 
//     std::cout << "Time is " << elapsed_seconds.count() << " seconds" << std::endl;
//     result.mx = l_x;
//     result.tx = l_xx;
//     return result;
// }

// void Constraints::process_iteration(int i, const Eigen::MatrixXd &state, const Eigen::VectorXd &poly_coeffs, 
//                        const Eigen::VectorXd &x_local_plan, Eigen::MatrixXd &l_x, 
//                        Eigen::Tensor<double, 3> &l_xx, const Constraints &constraints)
// {
//     std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();

//     Constraints non_const_constraints = constraints;
//     Uncertainty non_const_uncertainty = non_const_constraints.uncertainty_map;

//     Eigen::Vector2d closest_point_temp = non_const_constraints.find_closest_point(state.col(i), poly_coeffs, x_local_plan);

//     Eigen::Matrix<double, 4, 1> temp;
//     temp.row(0) << state(0, i) - closest_point_temp(0);
//     temp.row(1) << state(1, i) - closest_point_temp(1);
//     temp.row(2) << state(2, i) - non_const_constraints.param.desired_speed;
//     temp.row(3) << 0;

//     Eigen::Vector4d traj_cost = 2 * non_const_constraints.state_cost * temp;
//     Eigen::Vector4d l_x_i = traj_cost;

//     Eigen::Matrix4d l_xx_i = non_const_constraints.state_cost * 2;

//     struct_x_vx_mx_vmx obstacle_temple;
//     if (non_const_constraints.obstacles.size() != 0)
//     {
//         for (int j = 0; j < non_const_constraints.obstacles.size(); j++)
//         {
//             obstacle_temple = non_const_constraints.obstacles[j].get_obstalce_cost(i, state.col(i));
//             l_x_i = l_x_i + obstacle_temple.vx * non_const_constraints.param.w_obstacle;
//             l_xx_i = l_xx_i + obstacle_temple.mx * non_const_constraints.param.w_obstacle;
//         }
//     }
//     if (non_const_constraints.flag_uncertainty_map_use == true)
//     {
//         struct_x_vx_mx_vmx uncertianty_temp = non_const_uncertainty.get_uncertainty_cost(state.col(i));
//         l_x_i += uncertianty_temp.vx;
//         l_xx_i += uncertianty_temp.mx;
//     }

//     l_x.col(i) = l_x_i;

//     l_xx(0, 0, i) = l_xx_i(0, 0);
//     l_xx(0, 1, i) = l_xx_i(0, 1);
//     l_xx(0, 2, i) = l_xx_i(0, 2);
//     l_xx(0, 3, i) = l_xx_i(0, 3);
//     l_xx(1, 0, i) = l_xx_i(1, 0);
//     l_xx(1, 1, i) = l_xx_i(1, 1);
//     l_xx(1, 2, i) = l_xx_i(1, 2);
//     l_xx(1, 3, i) = l_xx_i(1, 3);
//     l_xx(2, 0, i) = l_xx_i(2, 0);
//     l_xx(2, 1, i) = l_xx_i(2, 1);
//     l_xx(2, 2, i) = l_xx_i(2, 2);
//     l_xx(2, 3, i) = l_xx_i(2, 3);
//     l_xx(3, 0, i) = l_xx_i(3, 0);
//     l_xx(3, 1, i) = l_xx_i(3, 1);
//     l_xx(3, 2, i) = l_xx_i(3, 2);
//     l_xx(3, 3, i) = l_xx_i(3, 3);

//     std::chrono::time_point<std::chrono::high_resolution_clock> current_time = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> elapsed_seconds = current_time - start_time; 
//     std::cout << "Time is " << elapsed_seconds.count() << " seconds" << std::endl;
// }

// struct_x_vx_mx_vmx Constraints::get_state_cost(const Eigen::MatrixXd &state, const Eigen::VectorXd &poly_coeffs, const Eigen::VectorXd &x_local_plan)
// {
//     struct_x_vx_mx_vmx result;

//     Eigen::MatrixXd l_x = Eigen::MatrixXd::Zero(this->param.num_states, this->param.horizon);
//     Eigen::Tensor<double, 3> l_xx(this->param.num_states, this->param.num_states, this->param.horizon);
//     l_xx.setZero();

//     std::vector<std::thread> threads;
//     std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
//     for (int i = 0; i < this->param.horizon; i++)
//     {
//         // 每次迭代启动一个新线程
//         threads.emplace_back(process_iteration, i, std::ref(state), std::ref(poly_coeffs), 
//                              std::ref(x_local_plan), std::ref(l_x), std::ref(l_xx), std::ref(*this));

//     }

//     // 等待所有线程完成
//     for (auto &t : threads)
//     {
//         t.join();
//     }
//     std::chrono::time_point<std::chrono::high_resolution_clock> current_time = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> elapsed_seconds = current_time - start_time; 
//     std::cout << "All Time is " << elapsed_seconds.count() << " seconds" << std::endl;

//     result.mx = l_x;
//     result.tx = l_xx;
//     return result;
// }


/*get_state_control_cost_derivate
input:0
output:l_x:2*4*n
function:return state_control_cost_derivate
*/
Eigen::Tensor<double, 3> Constraints::get_state_control_cost_dd()
{
    Eigen::Tensor<double, 3> l_ux(this->param.num_ctrls, this->param.num_states, this->param.horizon);
    l_ux.setZero();
    return l_ux;
}
//----------------------------------------------------------------------------
// obstacle

void Constraints::set_obstalces(const std::vector<Obstacle> &obstacles)
{
    this->obstacles = obstacles;
}
void Constraints::clear_obstacle()
{
    this->obstacles.clear();
}
// uncertainty map

void Constraints::set_uncertainty_map(const Uncertainty &uncertainty)
{
    this->uncertainty_map = uncertainty;
    this->flag_uncertainty_map_use = true;
}
void Constraints::clear_uncertainty_map()
{
    this->flag_uncertainty_map_use = false;
}

/*get J=x*Q*x+u*R*u
input:state,4*n;control 2*n;ploy_coeffs (ploy_order+1)*1,x_local_plan n*1
output: J
*/
double Constraints::get_J(const Eigen::MatrixXd &state, const Eigen::MatrixXd &control, const Eigen::VectorXd &poly_coeffs, const Eigen::VectorXd &x_local_plan)
{
    double J = 0;
    Eigen::Vector2d temp_point;
    Eigen::Vector4d diff_state;
    Eigen::MatrixXd matrix_temp;

    double x_cost = 0;
    double u_cost = 0;

    for (int i = 0; i < this->param.horizon; i++)
    {
        temp_point = this->find_closest_point(state.col(i), poly_coeffs, x_local_plan);
        diff_state << state(0, i) - temp_point(0), state(1, i) - temp_point(1), state(2, i) - this->param.desired_speed, state(3, i);

        matrix_temp = diff_state.transpose() * this->state_cost * diff_state;
        x_cost = matrix_temp(0, 0);

        matrix_temp = control.col(i).transpose() * this->control_cost * control.col(i);
        u_cost = matrix_temp(0, 0);
        // struct_x_vx_mx_vmx uncertianty_cost;
        // uncertianty_cost = this->uncertainty_map.get_uncertainty_cost(state.col(i));
        // std::cout<< uncertianty_cost.x<<std::endl;
        // J += x_cost + u_cost + uncertianty_cost.x  * this->param.w_uncertainty;
        J += x_cost + u_cost;
    }
    return J;
}

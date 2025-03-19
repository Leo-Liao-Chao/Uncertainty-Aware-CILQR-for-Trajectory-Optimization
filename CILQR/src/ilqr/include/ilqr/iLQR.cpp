#include "iLQR.h"

iLQR::iLQR(const Parameters &params)
    : params(params),
      localplanner(params),
      constraints(params),
      model(params)
{
    this->control_seq = Eigen::MatrixXd::Zero(params.num_ctrls, params.horizon);
    this->control_seq.row(0) = Eigen::RowVectorXd::Ones(params.horizon) * 0.5;

    int num_zeros = params.horizon/2; // 需要设置为 0 的元素个数
    Eigen::RowVectorXd row_vector = Eigen::RowVectorXd::Ones(params.horizon) * 0.1;
    row_vector.head(num_zeros) = Eigen::RowVectorXd::Zero(num_zeros); // 设置前 num_zeros 个值为 0
    this->control_seq.row(1) = row_vector;

    this->lamb_factor = 10;
    this->lamb_max = 10000;
}
void iLQR::set_Obstacle(const std::vector<Obstacle> &obstacles)
{
    this->constraints.set_obstalces(obstacles);
}
void iLQR::clear_Obstacle()
{
    this->constraints.clear_obstacle();
}
void iLQR::set_uncertainty_map(const Uncertainty &uncertainty)
{
    this->constraints.set_uncertainty_map(uncertainty);
}
void iLQR::clear_uncertainty_map()
{
    this->constraints.clear_uncertainty_map();
}

/*
input:2*n points array
function:set global path waypoints
*/
void iLQR::set_global_plan(const Eigen::MatrixXd &global_plan)
{
    this->global_plan = global_plan;
    this->localplanner.set_global_planner(global_plan);
}
/*
input:x_0:4*1 [x,y,v,theta].T U:2*n
output:X:4*(n);
function: make use of U and vehicle model to get new traj state;
*/
Eigen::MatrixXd iLQR::get_nominal_trajectory(const Eigen::VectorXd &x_0, const Eigen::MatrixXd &U)
{
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(this->params.num_states, this->params.horizon + 1);

    X.col(0) << x_0;

    for (int i = 0; i < this->params.horizon; i++)
    {
        X.col(i + 1) = this->model.forward_simulate(X.col(i), U.col(i));
    }
    return X;
}
/*
input: X:4*n U:2*n k:2*n K:2*4*n
output: (4+2)*(n+1)
function: based on X,U,k,K to get new X U;
*/
void iLQR::forward_pass(const Eigen::MatrixXd &X, const Eigen::MatrixXd &U, const Eigen::MatrixXd &k, const Eigen::Tensor<double, 3> &K)
{
    this->X_new = Eigen::MatrixXd::Zero(this->params.num_states, this->params.horizon + 1);
    this->X_new.col(0) << X.col(0);

    this->U_new = Eigen::MatrixXd::Zero(this->params.num_ctrls, this->params.horizon);

    Eigen::array<long, 3> extents_K = {2, 4, 1}; // 大小

    for (int i = 0; i < this->params.horizon; i++)
    {
        Eigen::array<long, 3> offsets = {0, 0, i}; // 起始位置
        Eigen::Tensor<double, 3> subTensor = K.slice(offsets, extents_K);
        Eigen::MatrixXd subMatrix = Eigen::Map<Eigen::MatrixXd>(subTensor.data(), subTensor.dimension(0), subTensor.dimension(1));

        this->U_new.col(i) = U.col(i) + k.col(i) + subMatrix * (this->X_new.col(i) - X.col(i)); // 2*1 + 2*1 + 2*4 x 4*1
        this->X_new.col(i + 1) = this->model.forward_simulate(X_new.col(i), this->U_new.col(i));
    }
}
/*
input:X:4*n U:2*n ploy_coeffs:(ploy_order+1)*1 x_local_plan:n*1 lamb:
function:caculate k K
*/
bool iLQR::backward_pass(const Eigen::MatrixXd &X, const Eigen::MatrixXd &U, const Eigen::VectorXd &ploy_coeffs, const Eigen::VectorXd &x_local_plan, const double &lamb)
{
    struct_x_vx_mx_vmx state_cost = this->constraints.get_state_cost(X, ploy_coeffs, x_local_plan);
    Eigen::MatrixXd l_x = state_cost.mx;
    Eigen::Tensor<double, 3> l_xx = state_cost.tx;

    struct_x_vx_mx_vmx control_cost = this->constraints.get_control_cost(X, U);
    Eigen::MatrixXd l_u = control_cost.mx;
    Eigen::Tensor<double, 3> l_uu = control_cost.tx;
    Eigen::Tensor<double, 3> l_ux = this->constraints.get_state_control_cost_dd();

    Eigen::VectorXd X_2_temp = X.block(2, 1, 1, this->params.horizon).transpose();
    Eigen::VectorXd X_3_temp = X.block(3, 1, 1, this->params.horizon).transpose();

    Eigen::Tensor<double, 3> df_dx = this->model.get_A_matrix(X_2_temp, X_3_temp, U.row(0));
    Eigen::Tensor<double, 3> df_du = this->model.get_B_matrix(X_3_temp);

    Eigen::VectorXd V_x = l_x.col(l_x.cols() - 1);

    Eigen::array<long, 3> offsets_V_xx = {0, 0, this->params.horizon - 1}; // 起始位置
    Eigen::array<long, 3> extents_V_xx = {4, 4, 1};                        // 大小
    Eigen::Tensor<double, 3> subTensor_l_xx = l_xx.slice(offsets_V_xx, extents_V_xx);
    Eigen::MatrixXd V_xx = Eigen::Map<Eigen::MatrixXd>(subTensor_l_xx.data(), subTensor_l_xx.dimension(0), subTensor_l_xx.dimension(1));

    Eigen::MatrixXd k = Eigen::MatrixXd::Zero(this->params.num_ctrls, this->params.horizon);
    Eigen::Tensor<double, 3> K(this->params.num_ctrls, this->params.num_states, this->params.horizon);
    K.setZero();

    Eigen::VectorXd Q_x;
    Eigen::VectorXd Q_u;

    Eigen::MatrixXd Q_xx;
    Eigen::MatrixXd Q_ux;
    Eigen::MatrixXd Q_uu;

    Eigen::array<long, 3> extents_df_dx = {4, 4, 1}; // 大小
    Eigen::array<long, 3> extents_df_du = {2, 4, 1}; // 大小
    Eigen::array<long, 3> extents_l_xx = {4, 4, 1};  // 大小
    Eigen::array<long, 3> extents_l_ux = {2, 4, 1};  // 大小
    Eigen::array<long, 3> extents_l_uu = {2, 2, 1};  // 大小
    Eigen::array<long, 3> extents_K = {2, 4, 1};     // 大小

    for (int j = this->params.horizon - 1; j >= 0; j--)
    {
        Eigen::array<long, 3> offsets = {0, 0, j}; // 起始位置
        // 获取子矩阵
        Eigen::Tensor<double, 3> subTensor_df_dx = df_dx.slice(offsets, extents_df_dx);
        Eigen::Tensor<double, 3> subTensor_df_du = df_du.slice(offsets, extents_df_du);
        Eigen::Tensor<double, 3> subTensor_l_xx = l_xx.slice(offsets, extents_l_xx);
        Eigen::Tensor<double, 3> subTensor_l_ux = l_ux.slice(offsets, extents_l_ux);
        Eigen::Tensor<double, 3> subTensor_l_uu = l_uu.slice(offsets, extents_l_uu);

        Eigen::MatrixXd subMatrix_df_dx = Eigen::Map<Eigen::MatrixXd>(subTensor_df_dx.data(), subTensor_df_dx.dimension(0), subTensor_df_dx.dimension(1));
        Eigen::MatrixXd subMatrix_df_du = Eigen::Map<Eigen::MatrixXd>(subTensor_df_du.data(), subTensor_df_du.dimension(0), subTensor_df_du.dimension(1));
        Eigen::MatrixXd subMatrix_l_xx = Eigen::Map<Eigen::MatrixXd>(subTensor_l_xx.data(), subTensor_l_xx.dimension(0), subTensor_l_xx.dimension(1));
        Eigen::MatrixXd subMatrix_l_ux = Eigen::Map<Eigen::MatrixXd>(subTensor_l_ux.data(), subTensor_l_ux.dimension(0), subTensor_l_ux.dimension(1));
        Eigen::MatrixXd subMatrix_l_uu = Eigen::Map<Eigen::MatrixXd>(subTensor_l_uu.data(), subTensor_l_uu.dimension(0), subTensor_l_uu.dimension(1));

        Q_x = l_x.col(j) + subMatrix_df_dx * V_x;                                     // 4*1 + 4*4 x 4*1 =4*1
        Q_u = l_u.col(j) + subMatrix_df_du * V_x;                                     // 2*1 + 2*4 x 4 *1 = 2*1
        Q_xx = subMatrix_l_xx + subMatrix_df_dx * V_xx * subMatrix_df_dx.transpose(); // 4*4 + 4*4 x 4*4 x 4*4 = 4*4
        Q_ux = subMatrix_l_ux + subMatrix_df_du * V_xx * subMatrix_df_dx.transpose(); // 2*4 + 2*4 x 4*4 x 4*4 = 2*4
        Q_uu = subMatrix_l_uu + subMatrix_df_du * V_xx * subMatrix_df_du.transpose(); // 2*2 + 2*4 x 4*4 x 4*2 = 2*2

        Eigen::EigenSolver<Eigen::MatrixXd> matrix_solver(Q_uu);

        matrix_solver.compute(Q_uu);

        if (matrix_solver.info() != Eigen::Success)
        {
            return false;
        }

        Eigen::VectorXd Q_uu_val = matrix_solver.eigenvalues().real();
        Eigen::MatrixXd Q_uu_vec = matrix_solver.eigenvectors().real();

        Q_uu_val = Q_uu_val.array().max(0);    // 小于0的为0；
        Q_uu_val = Q_uu_val.array() + lamb;    //+lamb
        Q_uu_val = Q_uu_val.array().inverse(); // 倒数

        Eigen::MatrixXd dig_Q_uu_val_inverse = Q_uu_val.asDiagonal();

        Eigen::MatrixXd Q_uu_inv;

        Q_uu_inv = Q_uu_vec * (dig_Q_uu_val_inverse * Q_uu_vec.transpose());

        k.col(j) = (-1) * Q_uu_inv * Q_u;                         // 2*2 x 2*1 = 2*1
        Eigen::Matrix<double, 2, 4> K_j = (-1) * Q_uu_inv * Q_ux; // 2*2 x 2*4 = 2*4

        V_x = Q_x - K_j.transpose() * Q_uu * k.col(j); // 4*1 + 4*2 x 2*2 x 2*1 = 4*1
        V_xx = Q_xx - K_j.transpose() * Q_uu * K_j;    // 4*4 +  4*2 x 2*2 x 2*4 = 4*4

        K(0, 0, j) = K_j(0, 0);
        K(0, 1, j) = K_j(0, 1);
        K(0, 2, j) = K_j(0, 2);
        K(0, 3, j) = K_j(0, 3);
        K(1, 0, j) = K_j(1, 0);
        K(1, 1, j) = K_j(1, 1);
        K(1, 2, j) = K_j(1, 2);
        K(1, 3, j) = K_j(1, 3);
    }
    this->k = k;
    this->K = K;
    return true;
}

/*
input:x_0:4*1 U:2*n  ploy_coeffs:(ploy_order+1)*1 x_local_plan:n*1
function:iteraction for X_result U_result
*/
void iLQR::get_optimal_control_seq(const Eigen::VectorXd &x_0, Eigen::MatrixXd &U, const Eigen::VectorXd &poly_coeffs, const Eigen::VectorXd &x_local_plan)
{
    Eigen::MatrixXd X = this->get_nominal_trajectory(x_0, U);
    double J_old = std::numeric_limits<double>::max();
    double lamb = 1;
    double J_new = 0;

    static int iteration_times ;
    iteration_times = 0;

    for (int i = 0; i < this->params.max_iterations; i++)
    {   iteration_times++;
        bool flag_backward = this->backward_pass(X, U, poly_coeffs, x_local_plan, lamb); // k K
        if (!flag_backward)
            break;
        this->forward_pass(X, U, this->k, this->K); // X_new U_new
        J_new = this->constraints.get_J(X, U, poly_coeffs, x_local_plan);

        if (J_new < J_old)
        {
            X = this->X_new;
            U = this->U_new;

            lamb = lamb / this->lamb_factor;
            if (std::abs(J_new - J_old) < this->params.tolerance)
            {
                break;
            }
        }
        else
        {
            lamb = lamb * this->lamb_factor;
            if (lamb > this->lamb_max)
            {
                break;
            }
        }
        J_old = J_new;
    }
    std::cout<<"Iteration times " << iteration_times <<"Max iteration times" <<this->params.max_iterations <<std::endl;
    std::cout<<"Iteration tolerance " << std::abs(J_new - J_old) <<"Max iteration tolerance" <<this->params.tolerance <<std::endl;
    std::cout<<"Iteration lamb " << lamb <<"Max lamb" <<this->lamb_max <<std::endl;
    this->X_result = X;
    this->U_result = U;
}

void iLQR::run_step(const Eigen::VectorXd &ego_state)
{
    this->localplanner.set_ego_state(ego_state);
    Eigen::MatrixXd ref_traj = this->localplanner.get_local_plan();
    Eigen::VectorXd ploy_coeffs = this->localplanner.get_local_plan_coeffs();

    this->get_optimal_control_seq(ego_state, this->control_seq, ploy_coeffs, ref_traj.row(0));
    this->ref_traj_result = ref_traj;
}

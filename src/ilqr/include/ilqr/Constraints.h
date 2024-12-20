#pragma once

#define EIGEN_USE_MKL_ALL
#include "mkl.h"

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>
#include <thread>
#include <future>
#include <vector>
#include "Parameters.h"
#include "Obstacle.h"
#include "struct_x_vx_mx_vmx.h"

#include "Uncertainty.h"



#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>

struct IterationResult
{
    int i;
    Eigen::Vector4d l_x_i ;
    Eigen::Matrix4d l_xx_i ;
};


class ThreadPool {
public:
    ThreadPool(size_t);
    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) 
        -> std::future<typename std::result_of<F(Args...)>::type>;
    ~ThreadPool();
private:
    std::vector< std::thread > workers;
    std::queue< std::function<void()> > tasks;
    
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
};

inline ThreadPool::ThreadPool(size_t threads)
    : stop(false)
{
    for(size_t i = 0;i<threads;++i)
        workers.emplace_back(
            [this]
            {
                for(;;)
                {
                    std::function<void()> task;

                    {
                        std::unique_lock<std::mutex> lock(this->queue_mutex);
                        this->condition.wait(lock,
                            [this]{ return this->stop || !this->tasks.empty(); });
                        if(this->stop && this->tasks.empty())
                            return;
                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }

                    task();
                }
            }
        );
}

template<class F, class... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args) 
    -> std::future<typename std::result_of<F(Args...)>::type>
{
    using return_type = typename std::result_of<F(Args...)>::type;

    auto task = std::make_shared< std::packaged_task<return_type()> >(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
        
    std::future<return_type> res = task->get_future();
    {
        std::unique_lock<std::mutex> lock(queue_mutex);

        if(stop)
            throw std::runtime_error("enqueue on stopped ThreadPool");

        tasks.emplace([task](){ (*task)(); });
    }
    condition.notify_one();
    return res;
}

inline ThreadPool::~ThreadPool()
{
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        stop = true;
    }
    condition.notify_all();
    for(std::thread &worker: workers)
        worker.join();
}


class Constraints
{
public:
    Constraints(const Parameters &param);

    // find_closest_point
    Eigen::Vector2d find_closest_point(const Eigen::Vector4d &state, const Eigen::VectorXd &coeffs, const Eigen::VectorXd &x_local_plan);
    // barrier function
    struct_x_vx_mx_vmx barrier_function(const double &q1, const double &q2, const double &c, const Eigen::Vector2d &c_dot);
    // get_control_cost_derivate
    struct_x_vx_mx_vmx get_control_cost(const Eigen::MatrixXd &state, const Eigen::MatrixXd &control);
    // get_state_cost_derivate
    struct_x_vx_mx_vmx get_state_cost(const Eigen::MatrixXd &state, const Eigen::VectorXd &poly_coeffs, const Eigen::VectorXd &x_local_plan);
    // static IterationResult process_iteration(int i, const Eigen::MatrixXd &state, const Eigen::VectorXd &poly_coeffs, 
    //                    const Eigen::VectorXd &x_local_plan,const Constraints &constraints);
    // static void process_iteration(int i, const Eigen::MatrixXd &state, const Eigen::VectorXd &poly_coeffs, 
    //                    const Eigen::VectorXd &x_local_plan, Eigen::MatrixXd &l_x, 
    //                    Eigen::Tensor<double, 3> &l_xx, const Constraints &constraints);

    // get_state_control_cost_derivate
    Eigen::Tensor<double, 3> get_state_control_cost_dd();

    // get_state_cost_derivate Obstacle
    void set_obstalces(const std::vector<Obstacle> &obstacles);
    void clear_obstacle();

    void set_uncertainty_map(const Uncertainty &uncertainty);
    void clear_uncertainty_map();

    // get J
    double get_J(const Eigen::MatrixXd &state, const Eigen::MatrixXd &control, const Eigen::VectorXd &poly_coeffs, const Eigen::VectorXd &x_local_plan);

private:
    Parameters param;
    Eigen::Matrix2d control_cost;
    Eigen::Matrix4d state_cost;

    std::vector<Obstacle> obstacles;

    Uncertainty uncertainty_map;
    bool flag_uncertainty_map_use;
};


#include "ARBIT.cuh"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <omp.h>

vector<pair<int, double>> thrust_propagateUncertainty(grid_map::GridMap vehicle_map_, int index, double sin_Vtheta_og, double cos_Vtheta_og, double sigma_x, double sigma_y, double sigma_theta)
{

    vector<double> times {0.0, 0.0, 0.0, 0.0};
    clock_t m_timer = clock();

    vector<double> ellipse_params;
    GridMapIterator it(vehicle_map_);

    for (unsigned int i = 0; i < index; ++i)
    {
        ++it;
    }

    vector<double> h_Cx, h_Cy;
    vector<Position> vec_pos;
    // thrust::device_vector<double> Cx, Cy;

    for (; !it.isPastEnd(); ++it){
        Position position;
        vehicle_map_.getPosition(*it, position);
        vec_pos.push_back(position);
        h_Cx.push_back(position.x());
        h_Cy.push_back(position.y());
    }

    int vec_size = h_Cx.size();

    // initilize of vector (high cost)
    thrust::device_vector<double> Cx = h_Cx, Cy = h_Cy;
    thrust::device_vector<double> sigma_x_i_vec(vec_size), sigma_y_i_vec(vec_size), rho(vec_size);

    // calculate uncertainty_error
    thrust::for_each(thrust::make_zip_iterator(thrust::make_tuple(Cx.begin(), Cy.begin(), sigma_x_i_vec.begin(), sigma_y_i_vec.begin(), rho.begin())),
                     thrust::make_zip_iterator(thrust::make_tuple(Cx.end(),   Cy.end(),   sigma_x_i_vec.end(),   sigma_x_i_vec.end(),   rho.end())),
                     thrust::make_zip_function(uncertainty_error_functor(sin_Vtheta_og, cos_Vtheta_og, sigma_x, sigma_y, sigma_theta)));
    
    times[0] = (double)(clock()-m_timer)/CLOCKS_PER_SEC;
    m_timer = clock();

    thrust::device_vector<double> a(vec_size), b(vec_size), c(vec_size);
    thrust::device_vector<double> half_major_axis(vec_size), half_minor_axis(vec_size), angle(vec_size);

    // calculate a b c
    thrust::for_each(thrust::make_zip_iterator(thrust::make_tuple(sigma_x_i_vec.begin(), sigma_y_i_vec.begin(), rho.begin(), a.begin(), b.begin(), c.begin())),
                     thrust::make_zip_iterator(thrust::make_tuple(sigma_x_i_vec.end(),   sigma_y_i_vec.end(),   rho.end(),   a.end(),   b.end(),   c.end())),
                     thrust::make_zip_function(abc_functor()));
    
    thrust::host_vector<double> vec_a = a, vec_b = b, vec_c = c;
    vector<int> major_index(vec_size), minor_index(vec_size);
    vector<Eigen::Matrix2f> D(vec_size), V(vec_size);

    for (int i = 0; i < vec_size; i++){
        Eigen::Matrix2f cov_i;
        
        double temp0 = vec_a[i], temp1 = vec_b[i], temp2 = vec_c[i]; //0.005571s
        
        cov_i << temp0, temp1, temp1, temp2; //0.005654s
        
        Eigen::EigenSolver<Eigen::Matrix2f> es(cov_i); //0.007130s
        
        
        D[i] = es.pseudoEigenvalueMatrix(); //0.003966s
        V[i] = es.pseudoEigenvectors();

        if (D[i](0,0) > D[i](1,1))
        {
            major_index[i] = 0;
            minor_index[i] = 1;
        } 
        else
        {
            major_index[i] = 1;
            minor_index[i] = 0;
        }
    }
    thrust::device_vector<int> thrust_major_index = major_index, thrust_minor_index = minor_index;
    thrust::device_vector<Eigen::Matrix2f> thrust_D = D, thrust_V = V;
    // calculate ellipse_params
    thrust::for_each(thrust::make_zip_iterator(thrust::make_tuple(a.begin(), b.begin(), c.begin(), thrust_major_index.begin(), thrust_minor_index.begin(), thrust_D.begin(), thrust_V.begin(), half_major_axis.begin(), half_minor_axis.begin(), angle.begin())),
                     thrust::make_zip_iterator(thrust::make_tuple(a.end(),   b.end(),   c.end(),   thrust_major_index.end(),   thrust_minor_index.end(),   thrust_D.end(),   thrust_V.end(),   half_major_axis.end(),   half_minor_axis.end(),   angle.end())),
                     thrust::make_zip_function(ellipse_params_functor()));
    
    // cout << abi::__cxa_demangle(typeid((*((&ellipse_params_vec[0]).get())).size()).name(),0,0,0) << endl << typeid(h_Cx).name() << endl;

    times[1] = (double)(clock()-m_timer)/CLOCKS_PER_SEC;
    m_timer = clock();
    
    thrust::host_vector<double> h_half_major_axis = half_major_axis, h_half_minor_axis = half_minor_axis, h_angle = angle;
    thrust::host_vector<double> h_sigma_x_i_vec = sigma_x_i_vec, h_sigma_y_i_vec = sigma_y_i_vec, h_rho = rho;

    double start_time = omp_get_wtime();

    vector<pair<int, double>> results(vec_size);
#pragma omp parallel for num_threads(8)
    // GridMapIterator it(vehicle_map_);
    for (int i = 0; i < vec_size; ++i){
        
        Position position = vec_pos[i];
        double params0 = h_half_major_axis[i], params1 = h_half_minor_axis[i], params2 = h_angle[i]; //0.003694s
        // vector<double> ellipse_params;
        // ellipse_params.push_back(params0);
        // ellipse_params.push_back(params1);
        // ellipse_params.push_back(params2);
        double sigma_x_i = h_sigma_x_i_vec[i], sigma_y_i = h_sigma_y_i_vec[i], rho = h_rho[i];//0.000544s
        double temp_Cx = h_Cx[i], temp_Cy = h_Cy[i];

        double numerator = 0;
        double denominator = 0;
        int count = 0;
        grid_map::EllipseIterator iterator(vehicle_map_, position, Length(2*params0,2*params1), params2);
        for (;
         !iterator.isPastEnd(); ++iterator)
        {
            double f_i;
            double Cxj, Cyj;

            Position position_j;
            vehicle_map_.getPosition(*iterator, position_j);
            Cxj = position_j.x();
            Cyj = position_j.y();

            f_i = nomal2(Cxj, Cyj, temp_Cx, temp_Cy, sigma_x_i, sigma_y_i, rho);

            numerator += f_i * vehicle_map_.atPosition("vehicle_map", position_j);
            denominator += f_i;
            count++;

        }

        results[i] = make_pair(count, numerator / denominator);

        // // Index temp_index;
        // // vehicle_map_.getIndex(position, temp_index);

        // // ROS_INFO("x %d, y %d", temp_index[0], temp_index[1]);
        // // visualMap("ellipse_map");

        // // ROS_INFO("numerator %f, denominator %f, uncertainty %f", numerator, denominator, numerator / denominator);
        // // if (denominator == 0) {
        // //     vehicle_map_.at("uncertainty_map", *it) = vehicle_map_.atPosition("vehicle_map", position);
        // // }
        
    }
    times[2] = omp_get_wtime() - start_time;
    ROS_INFO("times[0]:%lf,  times[1]:%lf,  times[2]:%lf,  times[3]:%lf", times[0], times[1], times[2], times[3]);   

    return results;

}


    // thrust::device_vector<double> u(vec_size), v(vec_size), t(vec_size);


    // thrust::device_vector<double> cos_Vtheta_og_vec(vec_size, cos_Vtheta_og);
    // thrust::device_vector<double> sin_Vtheta_og_vec(vec_size, sin_Vtheta_og);
    // thrust::device_vector<double> sigma_x_vec(vec_size, sigma_x);
    // thrust::device_vector<double> sigma_y_vec(vec_size, sigma_y);
    // thrust::device_vector<double> sigma_theta_vec(vec_size, sigma_theta);

    // // calculate u
    // thrust::for_each(thrust::make_zip_iterator(thrust::make_tuple(Cx.begin(), Cy.begin(), sin_Vtheta_og_vec.begin(), cos_Vtheta_og_vec.begin(), u.begin())),
    //                  thrust::make_zip_iterator(thrust::make_tuple(Cx.end(),   Cy.end(),   sin_Vtheta_og_vec.end(),   cos_Vtheta_og_vec.end(),   u.end())),
    //                  thrust::make_zip_function(u_functor()));

    // // calculate v
    // thrust::for_each(thrust::make_zip_iterator(thrust::make_tuple(Cx.begin(), Cy.begin(), sin_Vtheta_og_vec.begin(), cos_Vtheta_og_vec.begin(), v.begin())),
    //                  thrust::make_zip_iterator(thrust::make_tuple(Cx.end(),   Cy.end(),   sin_Vtheta_og_vec.end(),   cos_Vtheta_og_vec.end(),   v.end())),
    //                  thrust::make_zip_function(v_functor()));

    // // calculate t
    // thrust::for_each(thrust::make_zip_iterator(thrust::make_tuple(Cx.begin(), Cy.begin(), sin_Vtheta_og_vec.begin(), cos_Vtheta_og_vec.begin(), t.begin())),
    //                  thrust::make_zip_iterator(thrust::make_tuple(Cx.end(),   Cy.end(),   sin_Vtheta_og_vec.end(),   cos_Vtheta_og_vec.end(),   t.end())),
    //                  thrust::make_zip_function(t_functor()));

    // // calculate sigma_x_i
    // thrust::for_each(thrust::make_zip_iterator(thrust::make_tuple(sigma_x_vec.begin(), sigma_theta_vec.begin(), u.begin(), sigma_x_i_vec.begin())),
    //                  thrust::make_zip_iterator(thrust::make_tuple(sigma_x_vec.end(),   sigma_theta_vec.end(),   u.end(),   sigma_x_i_vec.end())),
    //                  thrust::make_zip_function(sigma_x_i_functor()));

    // // calculate sigma_y_i
    // thrust::for_each(thrust::make_zip_iterator(thrust::make_tuple(sigma_y_vec.begin(), sigma_theta_vec.begin(), v.begin(), sigma_y_i_vec.begin())),
    //                  thrust::make_zip_iterator(thrust::make_tuple(sigma_y_vec.end(),   sigma_theta_vec.end(),   v.end(),   sigma_y_i_vec.end())),
    //                  thrust::make_zip_function(sigma_y_i_functor()));

    // // calculate rho
    // thrust::for_each(thrust::make_zip_iterator(thrust::make_tuple(sigma_x_i_vec.begin(), sigma_y_i_vec.begin(),   sigma_theta_vec.begin(), t.begin(), rho.begin())),
    //                  thrust::make_zip_iterator(thrust::make_tuple(sigma_x_i_vec.end(),   sigma_y_i_vec.end(),     sigma_theta_vec.end(),   t.end(),   rho.end())),
    //                  thrust::make_zip_function(rho_functor()));



    // // calculate a
    // thrust::for_each(thrust::make_zip_iterator(thrust::make_tuple(sigma_x_i_vec.begin(), a.begin())),
    //                  thrust::make_zip_iterator(thrust::make_tuple(sigma_x_i_vec.end(),   a.end())),
    //                  thrust::make_zip_function(a_functor()));
                     
    // // calculate b
    // thrust::for_each(thrust::make_zip_iterator(thrust::make_tuple(sigma_x_i_vec.begin(), sigma_y_i_vec.begin(), rho.begin(), b.begin())),
    //                  thrust::make_zip_iterator(thrust::make_tuple(sigma_x_i_vec.end(),   sigma_y_i_vec.end(),   rho.end(),   b.end())),
    //                  thrust::make_zip_function(b_functor()));

    // // calculate c
    // thrust::for_each(thrust::make_zip_iterator(thrust::make_tuple(sigma_y_i_vec.begin(), c.begin())),
    //                  thrust::make_zip_iterator(thrust::make_tuple(sigma_y_i_vec.end(),   c.end())),
    //                  thrust::make_zip_function(c_functor()));
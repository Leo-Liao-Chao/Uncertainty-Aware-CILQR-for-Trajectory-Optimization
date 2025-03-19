// #define EIGEN_USE_MKL_ALL
// #define EIGEN_VECTORIZE_SSE4_2
// #include "mkl.h"

#include"../include/ilqr/ilqr_uncertainty_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_marker_publisher");
    ilqr_uncertainty_node ilqrplanner;
    ros::spin();
    return 0;
}

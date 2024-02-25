#include <Eigen/Geometry>
#include <iostream>
#include <sophus/se3.hpp>

int main(int argc, char** argv) {
    Eigen::Matrix<double, 6, 1> H = Eigen::Matrix<double, 6, 1>::Zero();
    H.head<3>() = Eigen::Vector3d(1, 2, 3);
    H.tail<3>() = Eigen::Vector3d(4, 5, 6);
    std::cout << H << std::endl;
    return 0;
}
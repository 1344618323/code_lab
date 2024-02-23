#include <Eigen/Geometry>
#include <iostream>
#include <sophus/se3.hpp>

int main(int argc, char** argv) {
    double dt = 0.1;
    Eigen::Vector3d omega(0, 0, M_PI);
    Sophus::SE3d pose;
    for (size_t i = 0; i < 20; i++) {
        // R = R exp(wt)
        pose.so3() = pose.so3() * Sophus::SO3d::exp(omega * dt);
    }
    std::cout << pose.unit_quaternion().coeffs() << std::endl; // 0,0,0,1

    for (size_t i = 0; i < 20; i++) {
        // q = q[1, 0.5wt]
        Eigen::Quaterniond q = pose.unit_quaternion() * Eigen::Quaterniond(1,
                                                                           0.5 * dt * omega[0],
                                                                           0.5 * dt * omega[1],
                                                                           0.5 * dt * omega[2]);
        q.normalize();
        pose.so3() = Sophus::SO3d(q);
    }
    std::cout << Eigen::Quaterniond(1, 0, 0, 0).angularDistance(pose.unit_quaternion())
              << std::endl; // a little delta 0.0509253 casued by quaternion normalize
    return 0;
}

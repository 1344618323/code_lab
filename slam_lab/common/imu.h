#pragma once
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

struct IMU {
    IMU() = default;
    IMU(double timestamp, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acce)
        : _timestamp(timestamp), _gyro(gyro), _acce(acce) {}
    double _timestamp = 0;
    Eigen::Vector3d _gyro;
    Eigen::Vector3d _acce;
};

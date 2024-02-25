#pragma once
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <sophus/se3.hpp>

namespace slam_lab {
constexpr double kDEG2RAD = M_PI / 180.0;  // deg->rad
constexpr double kRAD2DEG = 180.0 / M_PI;  // rad -> deg
constexpr double G_m_s2 = 9.81;            // 重力大小
}  // namespace slam_lab

struct IMU {
    IMU() = default;
    IMU(double timestamp, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acce)
        : _timestamp(timestamp), _gyro(gyro), _acce(acce) {}
    double _timestamp = 0;
    Eigen::Vector3d _gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d _acce = Eigen::Vector3d::Zero();
};

/// UTM 坐标
struct UTMCoordinate {
    UTMCoordinate() = default;
    explicit UTMCoordinate(int zone,
                           const Eigen::Vector2d& xy = Eigen::Vector2d::Zero(),
                           bool north = true)
        : _zone(zone), _xy(xy), _north(north) {}

    int _zone = 0;                                  // utm 区域
    Eigen::Vector2d _xy = Eigen::Vector2d::Zero();  // utm xy
    double _z = 0;                                  // z 高度（直接来自于gps）
    bool _north = true;                             // 是否在北半球
};

struct GNSS {
    enum StatusType {
        GNSS_FLOAT_SOLUTION = 5,         // cm~dm
        GNSS_FIXED_SOLUTION = 4,         // cm
        GNSS_PSEUDO_SOLUTION = 3,        // dm
        GNSS_SINGLE_POINT_SOLUTION = 2,  // 10m
        GNSS_NOT_EXIST = 0,              // no gnss signal
        GNSS_OTHER = -1,
    };

    GNSS() = default;
    GNSS(double timestamp,
         StatusType status,
         const Eigen::Vector3d& lat_lon_alt,
         double heading,
         double heading_valid)
        : _timestamp(timestamp),
          _status(status),
          _lat_lon_alt(lat_lon_alt),
          _heading(heading),
          _heading_valid(heading_valid) {}
    double _timestamp = 0;
    StatusType _status = StatusType::GNSS_NOT_EXIST;
    Eigen::Vector3d _lat_lon_alt = Eigen::Vector3d::Zero();
    double _heading = 0;
    bool _heading_valid = false;

    UTMCoordinate _utm;  // UTM 坐标（区域之类的也在内）
    bool _utm_valid = false;  // UTM 坐标是否已经计算（若经纬度给出错误数值，此处也为false）
    Sophus::SE3d _utm_pose;
};

struct WheelOdom {
    WheelOdom() = default;
    WheelOdom(double timestamp, double left_pulse, double right_pulse)
        : _timestamp(timestamp), _left_pulse(left_pulse), _right_pulse(right_pulse) {}

    double _timestamp = 0.0;
    double _left_pulse = 0.0;  // pulse num per second of left wheel
    double _right_pulse = 0.0;
};

struct NavState {
    NavState(double timestamp = 0.0,
             Eigen::Vector3d p = Eigen::Vector3d::Zero(),
             Sophus::SO3d R = Sophus::SO3d(),
             Eigen::Vector3d v = Eigen::Vector3d::Zero(),
             Eigen::Vector3d bg = Eigen::Vector3d::Zero(),
             Eigen::Vector3d ba = Eigen::Vector3d::Zero(),
             Eigen::Vector3d gravity = Eigen::Vector3d::Zero(),
             Eigen::Matrix3d p_cov = Eigen::Matrix3d::Zero())
        : _timestamp(timestamp),
          _p(p),
          _v(v),
          _R(R),
          _bg(bg),
          _ba(ba),
          _gravity(gravity),
          _p_cov(p_cov) {}
    Sophus::SE3d GetSE3() const { return Sophus::SE3d(_R, _p); }

    double _timestamp = 0.0;
    Eigen::Vector3d _p = Eigen::Vector3d::Zero();
    Eigen::Vector3d _v = Eigen::Vector3d::Zero();
    Sophus::SO3d _R;
    Eigen::Vector3d _bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d _ba = Eigen::Vector3d::Zero();
    Eigen::Vector3d _gravity = Eigen::Vector3d::Zero();
    Eigen::Matrix3d _p_cov = Eigen::Matrix3d::Zero();
};

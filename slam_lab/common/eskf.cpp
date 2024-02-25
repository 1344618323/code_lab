#include "eskf.h"
#include <glog/logging.h>

void ESKF::Config(const Option& option) {
    _motion_cov.diagonal() << 0, 0, 0, option._acce_var, option._acce_var, option._acce_var,
            option._gyro_var, option._gyro_var, option._gyro_var, option._bias_gyro_var,
            option._bias_gyro_var, option._bias_gyro_var, option._bias_acce_var,
            option._bias_acce_var, option._bias_acce_var, 0, 0, 0;
    _gnss_cov.diagonal() << option._gnss_pose_var, option._gnss_pose_var, option._gnss_height_var,
            option._gnss_ang_var, option._gnss_ang_var, option._gnss_ang_var;
    _option = option;
    _bg = _option._init_bg;
    _ba = _option._init_ba;
    _gravity = option._gravity;
    _wheel_odom_cov.diagonal() << option._wheel_odom_var, option._wheel_odom_var,
            option._wheel_odom_var;
    LOG(INFO) << "Config ESKF: bg = " << _bg.transpose() << " , ba = " << _ba.transpose()
              << " , gnss noise cov = " << _gnss_cov.diagonal().transpose()
              << " , motion noise cov = " << _motion_cov.diagonal().transpose()
              << " , gravity = " << _gravity.transpose() << " , gravity norm = " << _gravity.norm();
}

void ESKF::Start(double timestamp, const Eigen::Vector3d& origin_p, const Sophus::SO3d& origin_R) {
    _current_time = timestamp;
    _p = origin_p;
    _R = origin_R;
    _start = true;
}

bool ESKF::Predict(const IMU& imu) {
    double dt = imu._timestamp - _current_time;
    if (dt < 0 || !_start) {
        return false;
    }
    _current_time = imu._timestamp;

    // update error state
    Eigen::Matrix<double, 18, 18> F = Eigen::Matrix<double, 18, 18>::Identity();
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    F.block<3, 3>(3, 6) = -_R.matrix() * Sophus::SO3d::hat(imu._acce - _ba) * dt;
    F.block<3, 3>(3, 12) = -_R.matrix() * dt;
    F.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
    F.block<3, 3>(6, 6) = Sophus::SO3d::exp(-(imu._gyro - _bg) * dt).matrix();
    F.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity() * dt;
    _dx = F * _dx;  // useless line
    _cov = F * _cov * F.transpose() + _motion_cov;

    // update nominal state
    _p = _p + _v * dt + 0.5 * (_R * (imu._acce - _ba)) * dt * dt + 0.5 * _gravity * dt * dt;
    _v = _v + _R * (imu._acce - _ba) * dt + _gravity * dt;
    _R = _R * Sophus::SO3d::exp((imu._gyro - _bg) * dt);

    _last_imu = imu;
    return true;
}

bool ESKF::ObserveGNSS(const GNSS& gnss) {
    if (!_start || gnss._timestamp < _current_time) {
        return false;
    }
    if (!gnss._heading_valid) {
        return false;
    }
    _last_imu._timestamp = gnss._timestamp;
    Predict(_last_imu);
    ObserveSE3(gnss._utm_pose, _gnss_cov);
    return true;
}

bool ESKF::ObserveSE3(const Sophus::SE3d& pose, const Eigen::Matrix<double, 6, 6>& observe_cov) {
    if (!_start) {
        return false;
    }
    Eigen::Matrix<double, 6, 18> H = Eigen::Matrix<double, 6, 18>::Zero();
    H.block<3, 3>(0, 0).setIdentity();
    H.block<3, 3>(3, 6).setIdentity();

    Eigen::Matrix<double, 18, 6> K =
            _cov * H.transpose() * (H * _cov * H.transpose() + observe_cov).inverse();

    _cov = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * _cov;
    Eigen::Matrix<double, 6, 1> innov;
    innov.head<3>() = pose.translation() - _p;
    innov.tail<3>() = (_R.inverse() * pose.so3()).log();
    _dx = K * innov;

    UpdateAndReset();
    return true;
}

bool ESKF::ObserveWheelOdom(const WheelOdom& wheel_odom) {
    if (!_start || wheel_odom._timestamp < _current_time) {
        return false;
    }
    _last_imu._timestamp = wheel_odom._timestamp;
    Predict(_last_imu);

    Eigen::Matrix<double, 3, 18> H = Eigen::Matrix<double, 3, 18>::Zero();
    H.block<3, 3>(0, 3).setIdentity();

    Eigen::Matrix<double, 18, 3> K =
            _cov * H.transpose() * (H * _cov * H.transpose() + _wheel_odom_cov).inverse();

    // velocity obs
    double velo_l = _option._wheel_radius * wheel_odom._left_pulse / _option._wheel_circle_pulse *
                    2 * M_PI / _option._wheel_odom_span;
    double velo_r = _option._wheel_radius * wheel_odom._right_pulse / _option._wheel_circle_pulse *
                    2 * M_PI / _option._wheel_odom_span;
    double average_vel = 0.5 * (velo_l + velo_r);

    Eigen::Vector3d vel_odom(average_vel, 0.0, 0.0);
    Eigen::Vector3d vel_world = _R * vel_odom;

    _dx = K * (vel_world - _v);
    _cov = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * _cov;

    UpdateAndReset();
    return true;
}

void ESKF::UpdateAndReset() {
    _p += _dx.block<3, 1>(0, 0);
    _v += _dx.block<3, 1>(3, 0);
    _R = _R * Sophus::SO3d::exp(_dx.block<3, 1>(6, 0));
    _bg += _dx.block<3, 1>(9, 0);
    _ba += _dx.block<3, 1>(12, 0);
    _gravity += _dx.block<3, 1>(15, 0);

    // project cov
    Eigen::Matrix<double, 18, 18> J = Eigen::Matrix<double, 18, 18>::Identity();
    J.template block<3, 3>(6, 6) =
            Eigen::Matrix3d::Identity() - 0.5 * Sophus::SO3d::hat(_dx.block<3, 1>(6, 0));
    _cov = J * _cov * J.transpose();

    _dx.setZero();
}

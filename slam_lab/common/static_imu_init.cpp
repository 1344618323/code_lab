#include "static_imu_init.h"
#include <glog/logging.h>

bool StaticIMUInit::AddIMU(const IMU& imu) {
    if (_init_success) {
        return true;
    }
    if (_init_imu_deque.empty()) {
        _init_start_time = imu._timestamp;
    }
    _init_imu_deque.push_back(imu);
    if (imu._timestamp - _init_start_time > 10) {
        TryInit();
    }
    return false;
}

bool StaticIMUInit::TryInit() {
    Eigen::Vector3d mean_gyro, mean_acce;
    ComputeMeanAndCovDiag(
            _init_imu_deque, mean_gyro, _cov_gyro, [](const IMU& imu) { return imu._gyro; });
    ComputeMeanAndCovDiag(
            _init_imu_deque, mean_acce, _cov_acce, [](const IMU& imu) { return imu._acce; });
    _gravity = -mean_acce / mean_acce.norm() * slam_lab::G_m_s2;
    ComputeMeanAndCovDiag(_init_imu_deque, mean_acce, _cov_acce, [this](const IMU& imu) {
        return imu._acce + _gravity;
    });
    _init_bg = mean_gyro;
    _init_ba = mean_acce;
    _init_success = true;
    LOG(INFO) << "IMU init successfully: bg = " << _init_bg.transpose()
              << " , ba = " << _init_ba.transpose() << " , gyro sq = " << _cov_gyro.transpose()
              << " , gyro norm = " << _cov_gyro.norm() << " , acce sq = " << _cov_acce.transpose()
              << " , acce norm = " << _cov_acce.norm() << " , gravity = " << _gravity.transpose()
              << " , gravity norm = " << _gravity.norm();
    return true;
}

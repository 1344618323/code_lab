#include "imu_preintegration.h"
#include <glog/logging.h>

IMUPreintegration::IMUPreintegration(const Option& option)
    : _bg(option.init_bg), _ba(option.init_ba) {
    _gyro_acce_cov.diagonal() << option.gyro_var, option.gyro_var, option.gyro_var, option.acce_var,
            option.acce_var, option.acce_var;
}

void IMUPreintegration::Integrate(const IMU& imu, double dt) {
    Eigen::Vector3d gyro = imu._gyro - _bg;
    Eigen::Vector3d acce = imu._acce - _ba;

    // update preinteg measurement cov
    Eigen::Matrix<double, 9, 9> A;
    A.setIdentity();
    Eigen::Matrix<double, 9, 6> B;
    B.setZero();

    Sophus::SO3d deltaR = Sophus::SO3d::exp(gyro * dt);
    Eigen::Matrix3d rightJ = SO3rightJacobian(gyro * dt);
    Eigen::Matrix3d acce_hat = Sophus::SO3d::hat(acce);

    A.block<3, 3>(0, 0) = deltaR.matrix().transpose();
    A.block<3, 3>(3, 0) = -_dR.matrix() * dt * acce_hat;
    A.block<3, 3>(6, 0) = -0.5 * _dR.matrix() * acce_hat * dt * dt;
    A.block<3, 3>(6, 3) = dt * Eigen::Matrix3d::Identity();
    B.block<3, 3>(0, 0) = rightJ * dt;
    B.block<3, 3>(3, 3) = _dR.matrix() * dt;
    B.block<3, 3>(6, 3) = 0.5 * _dR.matrix() * dt * dt;

    _cov = A * _cov * A.transpose() + B * _gyro_acce_cov * B.transpose();

    // update the jacobian to update measurement by bias
    _dP_dba = _dP_dba + _dV_dba * dt - 0.5 * _dR.matrix() * dt * dt;
    _dP_dbg = _dP_dbg + _dV_dbg * dt - 0.5 * _dR.matrix() * dt * dt * acce_hat * _dR_dbg;
    _dV_dba = _dV_dba - _dR.matrix() * dt;
    _dV_dbg = _dV_dbg - _dR.matrix() * dt * acce_hat * _dR_dbg;
    _dR_dbg = deltaR.matrix().transpose() * _dR_dbg - rightJ * dt;

    // update preinteg measurement
    _dp = _dp + _dv * dt + 0.5 * _dR.matrix() * acce * dt * dt;
    _dv = _dv + _dR * acce * dt;
    _dR = _dR * deltaR;

    _dt += dt;
}

Sophus::SO3d IMUPreintegration::GetDeltaR(const Eigen::Vector3d& bg) const {
    return _dR * Sophus::SO3d::exp(_dR_dbg * (bg - _bg));
}
Eigen::Vector3d IMUPreintegration::GetDeltaP(const Eigen::Vector3d& bg,
                                             const Eigen::Vector3d& ba) const {
    return _dp + _dP_dbg * (bg - _bg) + _dP_dba * (ba - _ba);
}
Eigen::Vector3d IMUPreintegration::GetDeltaV(const Eigen::Vector3d& bg,
                                             const Eigen::Vector3d& ba) const {
    return _dv + _dV_dbg * (bg - _bg) + _dV_dba * (ba - _ba);
}

NavState IMUPreintegration::Predict(const NavState& start) const {
    Sophus::SO3d Rj = start._R * _dR;
    Eigen::Vector3d vj = start._R * _dv + start._v + start._gravity * _dt;
    Eigen::Vector3d pj =
            start._R * _dp + start._p + start._v * _dt + 0.5 * start._gravity * _dt * _dt;
    return NavState(start._timestamp + _dt, pj, Rj, vj, _bg, _ba, start._gravity);
}

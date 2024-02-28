#pragma once
#include "sensor_type.h"

class IMUPreintegration {
  public:
    struct Option {
        Eigen::Vector3d init_bg = Eigen::Vector3d::Zero();
        Eigen::Vector3d init_ba = Eigen::Vector3d::Zero();
        double gyro_var = 1e-4;
        double acce_var = 1e-4;
    };

    IMUPreintegration(const Option& option);
    void Integrate(const IMU& imu, double dt);
    Sophus::SO3d GetDeltaR(const Eigen::Vector3d& bg) const;
    Eigen::Vector3d GetDeltaP(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba) const;
    Eigen::Vector3d GetDeltaV(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba) const;
    NavState Predict(const NavState& start) const;
    Eigen::Vector3d GetInitBg() const { return _bg; }
    Eigen::Matrix<double, 9, 9> GetCov() const { return _cov; }
    double Getdt() const { return _dt; }

#define GetJacobian(state, bias) \
    Eigen::Matrix3d Get##state##bias() const { return _##state##_##bias; }

    GetJacobian(dR, dbg);
    GetJacobian(dV, dbg);
    GetJacobian(dV, dba);
    GetJacobian(dP, dbg);
    GetJacobian(dP, dba);

  private:
    // integral accumulation time
    double _dt = 0;

    // measurement
    Sophus::SO3d _dR;
    Eigen::Vector3d _dv = Eigen::Vector3d::Zero();
    Eigen::Vector3d _dp = Eigen::Vector3d::Zero();
    // init bias
    const Eigen::Vector3d _bg = Eigen::Vector3d::Zero();
    const Eigen::Vector3d _ba = Eigen::Vector3d::Zero();

    // measurement cov
    Eigen::Matrix<double, 9, 9> _cov = Eigen::Matrix<double, 9, 9>::Zero();

    // sensor noise
    Eigen::Matrix<double, 6, 6> _gyro_acce_cov = Eigen::Matrix<double, 6, 6>::Zero();

    // jacobian to update measurement by bias
    Eigen::Matrix3d _dR_dbg = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d _dV_dbg = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d _dV_dba = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d _dP_dbg = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d _dP_dba = Eigen::Matrix3d::Zero();
};
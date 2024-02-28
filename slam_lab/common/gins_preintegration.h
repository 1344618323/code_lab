#pragma once
#include "g2o_types.h"

class GinsIntegration {
  public:
    struct Option {
        IMUPreintegration::Option _imu_preintegration_option;
        Eigen::Vector3d _gravity = Eigen::Vector3d(0, 0, -slam_lab::G_m_s2);
        double _bias_gyro_var = 1e-12;
        double _bias_acce_var = 1e-8;
        double _gnss_pose_var = 1e-2;
        double _gnss_height_var = 1e-2;
        double _gnss_ang_var = 1.0 * slam_lab::kDEG2RAD * slam_lab::kDEG2RAD;
        double _wheel_odom_var = 0.25;
        double _wheel_odom_span = 0.1;        // 里程计测量间隔
        double _wheel_radius = 0.155;         // 轮子半径
        double _wheel_circle_pulse = 1024.0;  // 编码器每圈脉冲数
        bool _verbose = true;
    };
    GinsIntegration() = default;
    void Config(const Option& option);
    void AddIMU(const IMU& imu);
    void AddGNSS(const GNSS& gnss);
    NavState GetState() const;

  private:
    void Optimize();

    Option _option;
    double _current_time = 0;
    std::shared_ptr<IMUPreintegration> _imu_preintegration = nullptr;
    NavState _last_state, _cur_state;
    IMU _last_imu;
    GNSS _last_gnss, _cur_gnss;
    Eigen::Matrix<double, 6, 6> _gnss_info = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix3d _bg_rw_info = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d _ba_rw_info = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 15, 15> _prior_info = Eigen::Matrix<double, 15, 15>::Identity() * 1e2;
};
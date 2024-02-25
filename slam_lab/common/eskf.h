#pragma once
#include "sensor_type.h"

class ESKF {
  public:
    struct Option {
        double _gyro_var = 1e-5;
        double _acce_var = 1e-5;
        double _bias_gyro_var = 1e-6;
        double _bias_acce_var = 1e-4;

        double _gnss_pose_var = 1e-2;
        double _gnss_height_var = 1e-2;
        double _gnss_ang_var = 1.0 * slam_lab::kDEG2RAD;

        double _wheel_odom_var = 0.25;
        double _wheel_odom_span = 0.1;        // 里程计测量间隔
        double _wheel_radius = 0.155;         // 轮子半径
        double _wheel_circle_pulse = 1024.0;  // 编码器每圈脉冲数

        Eigen::Vector3d _init_bg = Eigen::Vector3d::Zero();
        Eigen::Vector3d _init_ba = Eigen::Vector3d::Zero();
        Eigen::Vector3d _gravity = Eigen::Vector3d(0, 0, -slam_lab::G_m_s2);
    };

    ESKF() = default;
    void Config(const Option& option);
    void Start(double timestamp,
               const Eigen::Vector3d& origin_p = Eigen::Vector3d::Zero(),
               const Sophus::SO3d& origin_R = Sophus::SO3d());
    bool Predict(const IMU& imu);
    bool ObserveGNSS(const GNSS& gnss);
    bool ObserveSE3(const Sophus::SE3d& pose, const Eigen::Matrix<double, 6, 6>& observe_cov);
    bool ObserveWheelOdom(const WheelOdom& wheel_odom);

    NavState GetNominalState() const {
        return NavState(_current_time, _p, _R, _v, _bg, _ba, _gravity, _cov.block<3, 3>(0, 0));
    }

  private:
    void UpdateAndReset();

    Option _option;
    double _current_time = 0.0;
    bool _start = false;

    // nominal state
    Eigen::Vector3d _p = Eigen::Vector3d::Zero();
    Eigen::Vector3d _v = Eigen::Vector3d::Zero();
    Sophus::SO3d _R;
    Eigen::Vector3d _bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d _ba = Eigen::Vector3d::Zero();
    Eigen::Vector3d _gravity = Eigen::Vector3d(0, 0, -slam_lab::G_m_s2);

    // error state
    Eigen::Matrix<double, 18, 1> _dx = Eigen::Matrix<double, 18, 1>::Zero();
    Eigen::Matrix<double, 18, 18> _cov = Eigen::Matrix<double, 18, 18>::Identity() * 1e-4;

    // noise cov
    Eigen::Matrix<double, 18, 18> _motion_cov =
            Eigen::Matrix<double, 18, 18>::Zero();  // diag(0_3, acce_var, gyro_var, bias_gyro_var,
                                                    // bias_acce_var, 0_3)
    Eigen::Matrix<double, 6, 6> _gnss_cov = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix3d _wheel_odom_cov = Eigen::Matrix3d::Zero();

    // record imu
    IMU _last_imu;
};

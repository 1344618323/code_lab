#include <iomanip>
#include <iostream>
#include "common/io_utils.h"
#include "common/utm_convert.h"

class IMUIntegration {
  public:
    IMUIntegration(const Eigen::Vector3d& gravity,
                   const Eigen::Vector3d& bg,
                   const Eigen::Vector3d& ba)
        : _gravity(gravity), _bg(bg), _ba(ba) {}

    void AddIMU(const IMU& imu) {
        double dt = imu._timestamp - _timestamp;
        if (dt > 0 && dt < 0.1) {
            _p = _p + _v * dt + 0.5 * (_R * (imu._acce - _ba)) * dt * dt + 0.5 * _gravity * dt * dt;
            _v = _v + _R * (imu._acce - _ba) * dt + _gravity * dt;
            _R = _R * Sophus::SO3d::exp((imu._gyro - _bg) * dt);
        }
        _timestamp = imu._timestamp;
    }

    NavState GetState() const { return NavState(_timestamp, _p, _R, _v, _bg, _ba, _gravity); }

  private:
    double _timestamp = 0;
    Eigen::Vector3d _gravity = Eigen::Vector3d(0, 0, -slam_lab::G_m_s2);
    Eigen::Vector3d _bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d _ba = Eigen::Vector3d::Zero();
    Sophus::SO3d _R;
    Eigen::Vector3d _p = Eigen::Vector3d::Zero();
    Eigen::Vector3d _v = Eigen::Vector3d::Zero();
};

int main(int argc, char** argv) {
    TxtIO io("/home/cxn/leofile/code_lab/slam_lab/data/imu_gnss_odom_10.txt");
    Eigen::Vector3d gravity(0, 0, -slam_lab::G_m_s2);
    Eigen::Vector3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
    Eigen::Vector3d init_ba(-0.165205, 0.0926887, 0.0058049);
    // Eigen::Vector3d init_bg = Eigen::Vector3d::Zero();
    // Eigen::Vector3d init_ba = Eigen::Vector3d::Zero();
    IMUIntegration imu_integration(gravity, init_bg, init_ba);
    Eigen::Vector3d gnss_origin = Eigen::Vector3d::Zero();
    Eigen::Vector2d antenna_pos(-0.17, -0.20);
    double antenna_angle = 12.06;
    bool gnss_inited = false;

    std::ofstream imu_fout("/home/cxn/leofile/code_lab/slam_lab/data_output/imu_integration_state.txt");
    std::ofstream gnss_fout("/home/cxn/leofile/code_lab/slam_lab/data_output/gnss_state.txt");

    auto save_result = [](std::ofstream& fout, const NavState& state) {
        auto save_vec3 = [&fout](const Eigen::Vector3d& v) {
            fout << v[0] << " " << v[1] << " " << v[2] << " ";
        };
        auto save_quat = [&fout](const Eigen::Quaterniond& q) {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };
        fout << std::setprecision(18) << state._timestamp << " " << std::setprecision(9);
        save_vec3(state._p);
        save_quat(state._R.unit_quaternion());
        save_vec3(state._v);
        save_vec3(state._bg);
        save_vec3(state._ba);
        save_vec3(state._gravity);
        fout << std::endl;
    };

    io.SetIMUProcessFun([&](const IMU& imu) {
          imu_integration.AddIMU(imu);
          save_result(imu_fout, imu_integration.GetState());
      })
            .SetGNSSProcessFun([&](const GNSS& gnss) {
                GNSS gnss_covert = gnss;
                if (!ConvertGps2UTM(gnss_covert, antenna_pos, antenna_angle)) {
                    return;
                };
                if (!gnss_inited) {
                    gnss_origin = gnss_covert._utm_pose.translation();
                    gnss_inited = true;
                }
                gnss_covert._utm_pose.translation() -= gnss_origin;
                NavState gnss_state(gnss_covert._timestamp,
                                    gnss_covert._utm_pose.translation(),
                                    gnss_covert._utm_pose.so3());
                save_result(gnss_fout, gnss_state);
            })
            .Go();
    return 0;
}

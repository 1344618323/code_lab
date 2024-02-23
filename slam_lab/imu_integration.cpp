#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#include <sophus/se3.hpp>
#include "common/io_utils.h"

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

    Sophus::SO3d GetR() const { return _R; }
    Eigen::Vector3d GetP() const { return _p; }
    Eigen::Vector3d GetV() const { return _v; }

  private:
    double _timestamp = 0;
    Eigen::Vector3d _gravity = Eigen::Vector3d(0, 0, -9.8);
    Eigen::Vector3d _bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d _ba = Eigen::Vector3d::Zero();
    Sophus::SO3d _R;
    Eigen::Vector3d _p = Eigen::Vector3d::Zero();
    Eigen::Vector3d _v = Eigen::Vector3d::Zero();
};

int main(int argc, char** argv) {
    TxtIO io("/home/cxn/leofile/slam_in_autonomous_driving/data/ch3/10.txt");
    Eigen::Vector3d gravity(0, 0, -9.8);
    Eigen::Vector3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
    Eigen::Vector3d init_ba(-0.165205, 0.0926887, 0.0058049);
    // Eigen::Vector3d init_bg = Eigen::Vector3d::Zero();
    // Eigen::Vector3d init_ba = Eigen::Vector3d::Zero();
    IMUIntegration imu_integration(gravity, init_bg, init_ba);

    std::ofstream fout("/home/cxn/leofile/code_lab/slam_lab/data/imu_integration_state.txt");
    auto save_result = [&fout](double timestamp,
                               const Sophus::SO3d& R,
                               const Eigen::Vector3d& v,
                               const Eigen::Vector3d& p) {
        auto save_vec3 = [&fout](const Eigen::Vector3d& v) {
            fout << v[0] << " " << v[1] << " " << v[2] << " ";
        };
        auto save_quat = [&fout](const Eigen::Quaterniond& q) {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };
        fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
        save_vec3(p);
        save_quat(R.unit_quaternion());
        save_vec3(v);
        fout << std::endl;
    };

    io.SetIMUProcessFun([&imu_integration, &save_result](const IMU& imu) {
          imu_integration.AddIMU(imu);
          save_result(imu._timestamp,
                      imu_integration.GetR(),
                      imu_integration.GetV(),
                      imu_integration.GetP());
      }).Go();
    return 0;
}

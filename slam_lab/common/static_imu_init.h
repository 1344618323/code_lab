#pragma once
#include <deque>
#include <numeric>
#include "sensor_type.h"

class StaticIMUInit {
  public:
    bool AddIMU(const IMU& imu);
    bool InitSuccess() const { return _init_success; }
    Eigen::Vector3d GetCovGyro() const { return _cov_gyro; }
    Eigen::Vector3d GetCovAcce() const { return _cov_acce; }
    Eigen::Vector3d GetInitBg() const { return _init_bg; }
    Eigen::Vector3d GetInitBa() const { return _init_ba; }
    Eigen::Vector3d GetGravity() const { return _gravity; }

  private:
    bool TryInit();

    template <typename C, typename D, typename Getter>
    void ComputeMeanAndCovDiag(const C& data, D& mean, D& cov_diag, Getter&& getter) {
        assert(data.size() > 1);
        mean = std::accumulate(data.begin(),
                               data.end(),
                               D::Zero().eval(),  // It seems that if `.eval()` is not written, it
                                                  // cannot be compiled, why?
                               [&getter](const D& sum, const auto& data) -> D {
                                   return sum + getter(data);
                               }) /
               data.size();
        cov_diag = std::accumulate(data.begin(),
                                   data.end(),
                                   D::Zero().eval(),
                                   [&getter, &mean](const D& sum, const auto& data) -> D {
                                       return sum + (getter(data) - mean).cwiseAbs2();
                                   }) /
                   (data.size() - 1);
    }

    bool _init_success = false;
    std::deque<IMU> _init_imu_deque;
    // noise cov
    Eigen::Vector3d _cov_gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d _cov_acce = Eigen::Vector3d::Zero();
    // random walk
    Eigen::Vector3d _init_bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d _init_ba = Eigen::Vector3d::Zero();

    Eigen::Vector3d _gravity = Eigen::Vector3d::Zero();

    double _init_start_time = 0.0;
};

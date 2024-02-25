#pragma once
#include <fstream>
#include "sensor_type.h"

class TxtIO {
  public:
    TxtIO(const std::string& file_path) : _fin(file_path) {}

    using IMUProcessFuncType = std::function<void(const IMU&)>;
    using GNSSProcessFuncType = std::function<void(const GNSS&)>;
    using WheelOdomProcessFuncType = std::function<void(const WheelOdom&)>;

    TxtIO& SetIMUProcessFun(IMUProcessFuncType imu_proc) {
        _imu_proc = std::move(imu_proc);
        return *this;
    }
    TxtIO& SetGNSSProcessFun(GNSSProcessFuncType gnss_proc) {
        _gnss_proc = std::move(gnss_proc);
        return *this;
    }
    TxtIO& SetWheelOdomProcessFun(WheelOdomProcessFuncType wheel_odom_prc) {
        _wheel_odom_proc = std::move(wheel_odom_prc);
        return *this;
    }

    void Go();

  private:
    std::fstream _fin;
    IMUProcessFuncType _imu_proc;
    GNSSProcessFuncType _gnss_proc;
    WheelOdomProcessFuncType _wheel_odom_proc;
};

#pragma once
#include <fstream>
#include "imu.h"

class TxtIO {
  public:
    TxtIO(const std::string& file_path) : _fin(file_path) {}

    using IMUProcessFunType = std::function<void(const IMU&)>;

    TxtIO& SetIMUProcessFun(IMUProcessFunType imu_proc) {
        _imu_proc = std::move(imu_proc);
        return *this;
    }

    void Go();

  private:
    std::fstream _fin;
    IMUProcessFunType _imu_proc;
};

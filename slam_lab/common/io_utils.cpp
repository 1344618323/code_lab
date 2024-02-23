#include "io_utils.h"

void TxtIO::Go() {
    if (!_fin) {
        return;
    }
    while (!_fin.eof()) {
        std::string line;
        std::getline(_fin, line);
        if (line.empty()) {
            continue;
        }
        // ignore comments
        if (line[0] == '#') {
            continue;
        }
        std::stringstream ss;
        ss << line;
        std::string data_type;
        ss >> data_type;
        if (data_type == "IMU" && _imu_proc) {
            double timestamp, gx, gy, gz, ax, ay, az;
            ss >> timestamp >> gx >> gy >> gz >> ax >> ay >> az;
            _imu_proc(IMU(timestamp, Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az)));
        }
    }
}

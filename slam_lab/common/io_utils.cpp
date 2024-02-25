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
        } else if (data_type == "GNSS" && _gnss_proc) {
            double timestamp, lat, lon, alt, heading;
            bool heading_valid;
            ss >> timestamp >> lat >> lon >> alt >> heading >> heading_valid;
            _gnss_proc(GNSS(timestamp,
                            GNSS::StatusType::GNSS_FIXED_SOLUTION,
                            Eigen::Vector3d(lat, lon, alt),
                            heading,
                            heading_valid));
        } else if (data_type == "ODOM" && _wheel_odom_proc) {
            double time, wl, wr;
            ss >> time >> wl >> wr;
            _wheel_odom_proc(WheelOdom(time, wl, wr));
        }
    }
}

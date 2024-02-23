#include <glog/logging.h>
#include <iomanip>
#include <sophus/se3.hpp>
#include "common/io_utils.h"
#include "common/static_imu_init.h"

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    TxtIO io("/home/cxn/leofile/slam_in_autonomous_driving/data/ch3/10.txt");
    StaticIMUInit imu_init;
    bool imu_inited = false;
    std::ofstream fout("/home/cxn/leofile/code_lab/slam_lab/data/eskf_gins_state.txt");
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

    io.SetIMUProcessFun([&imu_init, &imu_inited, &save_result](const IMU& imu) {
          if (!imu_init.InitSuccess()) {
              imu_init.AddIMU(imu);
              return;
          }
          if (!imu_inited) {
              imu_inited = true;
          }
      }).Go();
    return 0;
}
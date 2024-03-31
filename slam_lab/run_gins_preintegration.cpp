#include <glog/logging.h>
#include <iomanip>
#include "common/gins_preintegration.h"
#include "common/io_utils.h"
#include "common/static_imu_init.h"
#include "common/utm_convert.h"

DEFINE_bool(debug, false, "是否打印调试信息");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    TxtIO io("/home/cxn/leofile/code_lab/slam_lab/data/imu_gnss_odom_10.txt");
    StaticIMUInit imu_init;
    bool imu_inited = false;
    bool gnss_inited = false;
    Eigen::Vector3d gnss_origin;
    Eigen::Vector2d antenna_pos(-0.17, -0.20);
    double antenna_angle = 12.06;
    GinsIntegration gins_integ;

    std::ofstream fout("/home/cxn/leofile/code_lab/slam_lab/data/output/preinteg_gins_state.txt");
    auto save_result = [&fout](const NavState& state) {
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
          if (!imu_init.InitSuccess()) {
              imu_init.AddIMU(imu);
              return;
          }
          if (!imu_inited) {
              imu_inited = true;
              GinsIntegration::Option option;
              option._imu_preintegration_option.gyro_var = imu_init.GetCovGyro()[0];
              option._imu_preintegration_option.acce_var = imu_init.GetCovAcce()[0];
              option._imu_preintegration_option.init_bg = imu_init.GetInitBg();
              option._imu_preintegration_option.init_ba = imu_init.GetInitBa();
              option._gravity = imu_init.GetGravity();
              option._verbose = FLAGS_debug;
              gins_integ.Config(option);
          }
          if (!gnss_inited) {
              return;
          }
          gins_integ.AddIMU(imu);
          save_result(gins_integ.GetState());
      })
            .SetGNSSProcessFun([&](const GNSS& gnss) {
                if (!imu_inited) {
                    return;
                }
                GNSS gnss_covert = gnss;
                if (!ConvertGps2UTM(gnss_covert, antenna_pos, antenna_angle)) {
                    return;
                };
                if (!gnss_inited && gnss_covert._heading_valid) {
                    gnss_origin = gnss_covert._utm_pose.translation();
                    gnss_inited = true;
                }
                gnss_covert._utm_pose.translation() -= gnss_origin;
                gins_integ.AddGNSS(gnss_covert);
            })
            // .SetWheelOdomProcessFun([&](const WheelOdom& wheel_odom) {
            //     if (!gnss_inited || !imu_inited) {
            //         return;
            //     }
            //     eskf.ObserveWheelOdom(wheel_odom);
            // })
            .Go();
    return 0;
}
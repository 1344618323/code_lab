#include <glog/logging.h>
#include <iomanip>
#include "common/eskf.h"
#include "common/io_utils.h"
#include "common/static_imu_init.h"
#include "common/utm_convert.h"

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
    ESKF eskf;

    std::ofstream fout("/home/cxn/leofile/code_lab/slam_lab/data/output/eskf_gins_state.txt");
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

        // Eigen::Matrix2d p_cov2d = state._p_cov.block<2, 2>(0, 0);
        // fout << p_cov2d(0, 0) << " " << p_cov2d(0, 1) << " " << p_cov2d(1, 0) << " "
        //      << p_cov2d(1, 1);

        fout << std::endl;
    };

    io.SetIMUProcessFun([&](const IMU& imu) {
          if (!imu_init.InitSuccess()) {
              imu_init.AddIMU(imu);
              return;
          }
          if (!imu_inited) {
              imu_inited = true;
              ESKF::Option eskf_option;
              eskf_option._gyro_var = imu_init.GetCovGyro()[0];
              eskf_option._acce_var = imu_init.GetCovAcce()[0];
              eskf_option._init_bg = imu_init.GetInitBg();
              eskf_option._init_ba = imu_init.GetInitBa();
              eskf_option._gravity = imu_init.GetGravity();
              eskf.Config(eskf_option);
          }
          if (!gnss_inited) {
              return;
          }
          eskf.Predict(imu);
          save_result(eskf.GetNominalState());
      })
            .SetGNSSProcessFun([&](const GNSS& gnss) {
                if (!imu_inited) {
                    return;
                }
                GNSS gnss_covert = gnss;
                if (!ConvertGps2UTM(gnss_covert, antenna_pos, antenna_angle) ||
                    !gnss_covert._heading_valid) {
                    return;
                };
                if (!gnss_inited) {
                    gnss_origin = gnss_covert._utm_pose.translation();
                    gnss_inited = true;
                    gnss_covert._utm_pose.translation() -= gnss_origin;
                    eskf.Start(gnss_covert._timestamp,
                               gnss_covert._utm_pose.translation(),
                               gnss_covert._utm_pose.so3());
                } else {
                    gnss_covert._utm_pose.translation() -= gnss_origin;
                    eskf.ObserveGNSS(gnss_covert);
                }
            })
            .SetWheelOdomProcessFun([&](const WheelOdom& wheel_odom) {
                if (!gnss_inited || !imu_inited) {
                    return;
                }
                eskf.ObserveWheelOdom(wheel_odom);
            })
            .Go();
    return 0;
}
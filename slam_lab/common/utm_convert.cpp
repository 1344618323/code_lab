#include "utm_convert.h"
#include "utm_convert/utm.h"

bool LatLon2UTM(const Eigen::Vector2d& latlon, UTMCoordinate& utm_coor) {
    long zone = 0;
    char char_north = 0;
    long ret = Convert_Geodetic_To_UTM(latlon[0] * slam_lab::kDEG2RAD,
                                       latlon[1] * slam_lab::kDEG2RAD,
                                       &zone,
                                       &char_north,
                                       &utm_coor._xy[0],
                                       &utm_coor._xy[1]);
    utm_coor._zone = (int)zone;
    utm_coor._north = char_north == 'N';

    return ret == 0;
}

bool UTM2LatLon(const UTMCoordinate& utm_coor, Eigen::Vector2d& latlon) {
    bool ret = Convert_UTM_To_Geodetic((long)utm_coor._zone,
                                       utm_coor._north ? 'N' : 'S',
                                       utm_coor._xy[0],
                                       utm_coor._xy[1],
                                       &latlon[0],
                                       &latlon[1]);
    latlon *= slam_lab::kRAD2DEG;
    return ret == 0;
}

bool ConvertGps2UTM(GNSS& gps_msg,
                    const Eigen::Vector2d& antenna_pos,
                    const double& antenna_angle,
                    const Eigen::Vector3d& map_origin) {
    /// 经纬高转换为UTM
    UTMCoordinate utm_rtk;
    if (!LatLon2UTM(gps_msg._lat_lon_alt.head<2>(), utm_rtk)) {
        return false;
    }
    utm_rtk._z = gps_msg._lat_lon_alt[2];

    /// GPS heading 转成弧度
    double heading = 0;
    if (gps_msg._heading_valid) {
        heading = (90 - gps_msg._heading) * slam_lab::kDEG2RAD;  // 北东地转到东北天
    }

    /// TWG 转到 TWB
    Sophus::SE3d TBG(Sophus::SO3d::rotZ(antenna_angle * slam_lab::kDEG2RAD),
                     Eigen::Vector3d(antenna_pos[0], antenna_pos[1], 0));
    Sophus::SE3d TGB = TBG.inverse();

    /// 若指明地图原点，则减去地图原点
    double x = utm_rtk._xy[0] - map_origin[0];
    double y = utm_rtk._xy[1] - map_origin[1];
    double z = utm_rtk._z - map_origin[2];
    Sophus::SE3d TWG(Sophus::SO3d::rotZ(heading), Eigen::Vector3d(x, y, z));
    Sophus::SE3d TWB = TWG * TGB;

    gps_msg._utm_valid = true;
    gps_msg._utm._xy[0] = TWB.translation().x();
    gps_msg._utm._xy[1] = TWB.translation().y();
    gps_msg._utm._z = TWB.translation().z();

    if (gps_msg._heading_valid) {
        // 组装为带旋转的位姿
        gps_msg._utm_pose = TWB;
    } else {
        // 组装为仅有平移的SE3
        // 注意当安装偏移存在时，并不能实际推出车辆位姿
        gps_msg._utm_pose = Sophus::SE3d(Sophus::SO3d(), TWB.translation());
    }

    return true;
}

bool ConvertGps2UTMOnlyTrans(GNSS& gps_msg) {
    /// 经纬高转换为UTM
    UTMCoordinate utm_rtk;
    LatLon2UTM(gps_msg._lat_lon_alt.head<2>(), utm_rtk);
    gps_msg._utm_valid = true;
    gps_msg._utm._xy = utm_rtk._xy;
    gps_msg._utm._z = gps_msg._lat_lon_alt[2];
    gps_msg._utm_pose = Sophus::SE3d(
            Sophus::SO3d(),
            Eigen::Vector3d(gps_msg._utm._xy[0], gps_msg._utm._xy[1], gps_msg._utm._z));
    return true;
}
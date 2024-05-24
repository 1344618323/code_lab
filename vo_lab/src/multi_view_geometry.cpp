#include "multi_view_geometry.h"
#include <glog/logging.h>

namespace vo_lab {
Eigen::Vector3d MultiViewGeometry::triangulate(const Sophus::SE3d& T21,
                                               const Eigen::Vector3d& bv1,
                                               const Eigen::Vector3d& bv2) {
    std::vector<cv::Point2f> lpt = {cv::Point2f(bv1.x() / bv1.z(), bv1.y() / bv1.z())};
    std::vector<cv::Point2f> rpt = {cv::Point2f(bv2.x() / bv2.z(), bv2.y() / bv2.z())};
    cv::Matx34f P0 = cv::Matx34f(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    Eigen::Matrix3d R = T21.rotationMatrix();
    Eigen::Vector3d t = T21.translation();
    cv::Matx34f P1 = cv::Matx34f(R(0, 0),
                                 R(0, 1),
                                 R(0, 2),
                                 t(0),
                                 R(1, 0),
                                 R(1, 1),
                                 R(1, 2),
                                 t(1),
                                 R(2, 0),
                                 R(2, 1),
                                 R(2, 2),
                                 t(2));
    cv::Mat pts;
    cv::triangulatePoints(P0, P1, lpt, rpt, pts);
    pts.col(0) /= pts.col(0).at<float>(3);
    return Eigen::Vector3d(
            pts.col(0).at<float>(0), pts.col(0).at<float>(1), pts.col(0).at<float>(2));
}

bool MultiViewGeometry::triangulate(const std::vector<Sophus::SE3d>& Tcw,
                                    const std::vector<Eigen::Vector3d>& bv,
                                    Eigen::Vector3d& pw) {
    Eigen::MatrixXd A(2 * Tcw.size(), 4);
    Eigen::VectorXd b(2 * Tcw.size());
    b.setZero();
    for (size_t i = 0; i < Tcw.size(); i++) {
        Eigen::Matrix<double, 3, 4> m = Tcw[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = -bv[i][2] * m.row(1) + bv[i][1] * m.row(2);
        A.block<1, 4>(2 * i + 1, 0) = bv[i][2] * m.row(0) - bv[i][0] * m.row(2);
    }
    auto svd = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pw = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();
    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        // pw has only 3 degrees of freedom
        return true;
    }
    return false;
}

float MultiViewGeometry::computeSampsonDistance(const Eigen::Matrix3d& F21,
                                                const cv::Point2f& p1,
                                                const cv::Point2f& p2) {
    Eigen::Vector3d pt1(p1.x, p1.y, 1.);
    Eigen::Vector3d pt2(p2.x, p2.y, 1.);
    float num = pt2.transpose() * F21 * pt1;
    float den =
            (F21.transpose() * pt2).head<2>().squaredNorm() + (F21 * pt1).head<2>().squaredNorm();
    return std::sqrt(num * num / den);

    // Implement the same functionality using opencv API
    // cv::Mat F;
    // cv::eigen2cv(Frl, F);
    // cv::Point3d pt1(p1.x, p1.y, 1.);
    // cv::Point3d pt2(p2.x, p2.y, 1.);
    // // the three input of `cv::sampsonDistance` must be CV_64F!
    // // and std::vector<cv::Point3d> isn't a `InputArray`.
    // return std::sqrt(cv::sampsonDistance(cv::Mat(pt1), cv::Mat(pt2), F));
}

bool MultiViewGeometry::essentialMatrix(const std::vector<Eigen::Vector3d>& bv1,
                                        const std::vector<Eigen::Vector3d>& bv2,
                                        Eigen::Matrix3d& R21,
                                        Eigen::Vector3d& t21,
                                        float f,
                                        float errth,
                                        bool optimize,
                                        std::vector<size_t>& voutlier_idx) {
    if (bv1.size() != bv2.size() || bv1.size() < 8) {
        return false;
    }
    voutlier_idx.reserve(bv1.size());
    std::vector<cv::Point2f> lpx, rpx;
    lpx.reserve(bv1.size());
    rpx.reserve(bv2.size());
    for (size_t i = 0; i < bv1.size(); i++) {
        lpx.emplace_back(bv1[i].x() / bv1[i].z(), bv1[i].y() / bv1[i].z());
        rpx.emplace_back(bv2[i].x() / bv2[i].z(), bv2[i].y() / bv2[i].z());
    }

    float confidence = optimize ? 0.999 : 0.99;
    cv::Mat inliers;
    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    /*
    although
    https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gab705726dc6b655acf50bc936942824ef states
    `cv::findEssentialMat` requires at least 5 pairs of points, but
    https://answers.opencv.org/question/136092/findessentialmat-returns-3x30-or-3x12-mat/ discovered
    an issue: using 5 pairs of points may solve for multiple `E` mat, so adjust the input condition
    to 8
    */
    cv::Mat E = cv::findEssentialMat(lpx, rpx, K, cv::RANSAC, confidence, errth / f, inliers);
    for (size_t i = 0; i < lpx.size(); i++) {
        if (!inliers.at<uchar>(i)) {
            voutlier_idx.push_back(i);
        }
    }
    cv::Mat R, t;
    cv::recoverPose(E, lpx, rpx, K, R, t, inliers);
    cv::cv2eigen(R, R21);
    cv::cv2eigen(t, t21);
    return true;
}

Eigen::Matrix3d MultiViewGeometry::fundamentalMatrix(const Sophus::SE3d& Twc1,
                                                     const Sophus::SE3d& Twc2,
                                                     const Eigen::Matrix3d& K1,
                                                     const Eigen::Matrix3d& K2) {
    Sophus::SE3d T12 = Twc1.inverse() * Twc2;
    // calc F12
    return K1.inverse().transpose() * Sophus::SO3d::hat(T12.translation()) * T12.rotationMatrix() *
           K2.inverse();
}
}  // namespace vo_lab
#include "multi_view_geometry.h"

namespace vo_lab {
Eigen::Vector3d MultiViewGeometry::triangulate(const Sophus::SE3d& Trl,
                                               const Eigen::Vector3d& bvl,
                                               const Eigen::Vector3d& bvr) {
    std::vector<cv::Point2f> lpt = {cv::Point2f(bvl.x() / bvl.z(), bvl.y() / bvl.z())};
    std::vector<cv::Point2f> rpt = {cv::Point2f(bvr.x() / bvr.z(), bvr.y() / bvr.z())};
    cv::Matx34f P0 = cv::Matx34f(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    Eigen::Matrix3d R = Trl.rotationMatrix();
    Eigen::Vector3d t = Trl.translation();
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
                                    const std::vector<Eigen::Vector3d>& pc,
                                    Eigen::Vector3d& pw) {
    Eigen::MatrixXd A(2 * Tcw.size(), 4);
    Eigen::VectorXd b(2 * Tcw.size());
    b.setZero();
    for (size_t i = 0; i < Tcw.size(); i++) {
        Eigen::Matrix<double, 3, 4> m = Tcw[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = -pc[i][2] * m.row(1) + pc[i][1] * m.row(2);
        A.block<1, 4>(2 * i + 1, 0) = pc[i][2] * m.row(0) - pc[i][0] * m.row(2);
    }
    auto svd = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pw = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();
    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        // pw has only 3 degrees of freedom
        return true;
    }
    return false;
}

float MultiViewGeometry::computeSampsonDistance(const Eigen::Matrix3d& Frl,
                                                const cv::Point2f& leftpt,
                                                const cv::Point2f& rightpt) {
    Eigen::Vector3d lpt(leftpt.x, leftpt.y, 1.);
    Eigen::Vector3d rpt(rightpt.x, rightpt.y, 1.);
    float num = rpt.transpose() * Frl * lpt;
    float den =
            (Frl.transpose() * rpt).head<2>().squaredNorm() + (Frl * lpt).head<2>().squaredNorm();
    return std::sqrt(num * num / den);

    // Implement the same functionality using opencv API
    // cv::Mat F;
    // cv::eigen2cv(Frl, F);
    // cv::Point3d lpt(leftpt.x, leftpt.y, 1.);
    // cv::Point3d rpt(rightpt.x, rightpt.y, 1.);
    // // the three input of `cv::sampsonDistance` must be CV_64F!
    // // and std::vector<cv::Point3d> isn't a `InputArray`.
    // return std::sqrt(cv::sampsonDistance(cv::Mat(lpt), cv::Mat(rpt), F));
}

bool MultiViewGeometry::essentialMatrix(const std::vector<Eigen::Vector3d>& bvl,
                                        const std::vector<Eigen::Vector3d>& bvr,
                                        Eigen::Matrix3d& Rrl,
                                        Eigen::Vector3d& trl,
                                        float f,
                                        float errth,
                                        bool optimize,
                                        std::vector<size_t>& voutlier_idx) {
    if (bvl.size() != bvr.size() || bvl.size() < 5) {
        return false;
    }
    voutlier_idx.reserve(bvl.size());
    std::vector<cv::Point2f> lpx, rpx;
    lpx.reserve(bvl.size());
    rpx.reserve(bvr.size());
    for (size_t i = 0; i < bvl.size(); i++) {
        lpx.emplace_back(bvl[i].x() / bvl[i].z(), bvl[i].y() / bvl[i].z());
        rpx.emplace_back(bvr[i].x() / bvr[i].z(), bvr[i].y() / bvr[i].z());
    }

    float confidence = optimize ? 0.999 : 0.99;
    cv::Mat inliers;
    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    // this API need at least 5 pts
    cv::Mat E = cv::findEssentialMat(lpx, rpx, K, cv::RANSAC, confidence, errth / f, inliers);
    for (size_t i = 0; i < lpx.size(); i++) {
        if (!inliers.at<uchar>(i)) {
            voutlier_idx.push_back(i);
        }
    }
    cv::Mat R, t;
    cv::recoverPose(E, lpx, rpx, K, R, t, inliers);
    cv::cv2eigen(R, Rrl);
    cv::cv2eigen(t, trl);
    return true;
}
}  // namespace vo_lab
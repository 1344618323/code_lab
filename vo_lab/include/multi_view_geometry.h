#pragma once
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

namespace vo_lab {
class MultiViewGeometry {
  public:
    static Eigen::Vector3d triangulate(const Sophus::SE3d& T21,
                                       const Eigen::Vector3d& bv1,
                                       const Eigen::Vector3d& bv2);
    static bool triangulate(const std::vector<Sophus::SE3d>& Tcw,
                            const std::vector<Eigen::Vector3d>& bv,
                            Eigen::Vector3d& pw);
    static float computeSampsonDistance(const Eigen::Matrix3d& F21,
                                        const cv::Point2f& p1,
                                        const cv::Point2f& p2);
    static bool essentialMatrix(const std::vector<Eigen::Vector3d>& bv1,
                                const std::vector<Eigen::Vector3d>& bv2,
                                Eigen::Matrix3d& R21,
                                Eigen::Vector3d& t21,
                                float f,
                                float errth,
                                bool optimize,
                                std::vector<size_t>& voutlier_idx);
    static Eigen::Matrix3d fundamentalMatrix(const Sophus::SE3d& Twcl,
                                             const Sophus::SE3d& Twc2,
                                             const Eigen::Matrix3d& K1,
                                             const Eigen::Matrix3d& K2);
    static bool P3PRansac(const std::vector<Eigen::Vector3d>& bv,
                          const std::vector<Eigen::Vector3d>& wpt,
                          int maxiter,
                          float errth,
                          bool optimize,
                          float f,
                          Sophus::SE3d& Twc,
                          std::vector<size_t>& voutlier_idx);
    static bool ceresPnP(
            const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& unpt,
            const std::vector<Eigen::Vector3d>& wpt,
            Sophus::SE3d& Twc,
            int maxiter,
            std::vector<size_t>& voutlier_idx);
};
}  // namespace vo_lab
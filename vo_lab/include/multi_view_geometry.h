#pragma once
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

namespace vo_lab {
class MultiViewGeometry {
  public:
    static Eigen::Vector3d triangulate(const Sophus::SE3d& Trl,
                                       const Eigen::Vector3d& bvl,
                                       const Eigen::Vector3d& bvr);
    static bool triangulate(const std::vector<Sophus::SE3d>& Twc,
                            const std::vector<Eigen::Vector3d>& bv,
                            Eigen::Vector3d& wpt);
    static float computeSampsonDistance(const Eigen::Matrix3d& Frl,
                                        const cv::Point2f& leftpt,
                                        const cv::Point2f& rightpt);
    static bool essentialMatrix(const std::vector<Eigen::Vector3d>& bvl,
                                const std::vector<Eigen::Vector3d>& bvr,
                                Eigen::Matrix3d& Rrl,
                                Eigen::Vector3d& trl,
                                float f,
                                float errth,
                                bool optimize,
                                std::vector<size_t>& voutlier_idx);
};
}  // namespace vo_lab
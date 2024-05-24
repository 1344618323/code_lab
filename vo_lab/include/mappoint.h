#pragma once
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "frame.h"
namespace vo_lab {
class Mappoint {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Mappoint> Ptr;
    Mappoint(const uint64_t mpid, const Frame::Ptr anch_kf, const cv::Mat& desc = cv::Mat());
    void setPos(const Eigen::Vector3d& pos);
    bool getPos(Eigen::Vector3d& pos) const;
    bool is3D() const;
    void addKfObs(const Frame::Ptr kf, const cv::Mat& desc = cv::Mat());
    void removeKfObs(const Frame::Ptr kf);
    std::map<uint64_t, Frame::Ptr> getKfObs() const;

    uint64_t mpid;

  private:
    mutable std::mutex _mp_mutex;

    // anch kf
    Frame::Ptr _anch_kf;

    bool _is_3d = false;
    Eigen::Vector3d _pos;

    std::map<uint64_t, Frame::Ptr> _kfs;
    std::map<Frame::Ptr, cv::Mat> _map_kf_desc;
};
}  // namespace vo_lab
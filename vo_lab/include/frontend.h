#pragma once

#include "feature_extractor.h"
#include "frame.h"
#include "map.h"

namespace vo_lab {

class MotionModel {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void predict(Sophus::SE3d& Twc, double time);
    void update(const Sophus::SE3d& Twc, double time);

  private:
    double _prev_time = -1;
    Sophus::SE3d _prev_Twc;
    Eigen::Matrix<double, 6, 1> _log_relT;
};

class Frontend {
  public:
    typedef std::shared_ptr<Frontend> Ptr;
    Frontend(const Config& config,
             const Map::Ptr map,
             const cv::Ptr<cv::CLAHE> clahe,
             const FeatureExtractor::Ptr extractor,
             const FeatureTracker::Ptr tracker);
    bool tracking(const Frame::Ptr curframe);

  private:
    void preprocessFrame(const Frame::Ptr curframe);
    void kltTracking(const Frame::Ptr curframe);
    void epipolar2d2dFiltering(const Frame::Ptr curframe);
    void computePose(const Frame::Ptr curframe);
    void inheritOldKeypoint(const Frame::Ptr kf);
    bool createNewKeypoint(const Frame::Ptr kf);

    uint64_t _mpid = 0;
    const Config& _config;
    const Map::Ptr _map;
    const cv::Ptr<cv::CLAHE> _clahe;
    const FeatureExtractor::Ptr _extractor;
    const FeatureTracker::Ptr _tracker;

    Frame::ConstPtr _prevframe = nullptr;
    Frame::ConstPtr _prev_kf = nullptr;
    MotionModel _motion_model;
};
}  // namespace vo_lab
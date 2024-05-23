#pragma once
#include <opencv2/opencv.hpp>

#include "backend.h"
#include "config.h"
#include "feature_tracker.h"
#include "frame.h"
#include "frontend.h"
#include "map.h"
#include "viewer.h"

namespace vo_lab {

class VO {
  public:
    VO(const Config& config);
    ~VO();
    void addImage(double time, cv::Mat& im_left, cv::Mat& im_right);

  private:
    const Config& _config;
    Camera::Ptr _cam_left = nullptr;
    Camera::Ptr _cam_right = nullptr;
    cv::Ptr<cv::CLAHE> _clahe = nullptr;
    Frontend::Ptr _frontend = nullptr;
    Backend::Ptr _backend = nullptr;
    Viewer::Ptr _viewer = nullptr;
    Map::Ptr _map = nullptr;
    FeatureTracker::Ptr _tracker = nullptr;
    FeatureExtractor::Ptr _extractor = nullptr;
    uint64_t _frame_id = 0;
};

}  // namespace vo_lab
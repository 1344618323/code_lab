#pragma once
#include <Eigen/Eigen>
#include <atomic>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <thread>
#include "map.h"

namespace vo_lab {
class Backend {
  public:
    typedef std::shared_ptr<Backend> Ptr;
    Backend(const Config& config,
            const Map::Ptr map,
            const cv::Ptr<cv::CLAHE> clahe,
            const FeatureTracker::Ptr tracker);
    ~Backend();
    void addKeyframe(const Frame::Ptr kf);

  private:
    void mapping();
    void stereoMatching(const Frame::Ptr kf);
    void triangulateStereo(const Frame::Ptr kf);

    const Config& _config;
    const Map::Ptr _map;
    const cv::Ptr<cv::CLAHE> _clahe;
    const FeatureTracker::Ptr _tracker;

    std::thread _mapping_thread;
    std::thread _estimator_thread;
    std::atomic<bool> _stop = false;

    std::condition_variable _kf_cv;
    std::mutex _kf_mutex;
    std::queue<std::shared_ptr<Frame>> _kfs;
};
}  // namespace vo_lab
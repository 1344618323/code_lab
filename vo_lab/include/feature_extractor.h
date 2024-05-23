#pragma once

#include <opencv2/opencv.hpp>

namespace vo_lab {
class FeatureExtractor {
  public:
    typedef std::shared_ptr<FeatureExtractor> Ptr;
    FeatureExtractor(int cell_size, double max_quality)
        : _cell_size(cell_size), _max_quality(max_quality), _brief(cv::ORB::create()) {}

    void detect(const cv::Mat& im,
                const std::vector<cv::Point2f>& curkps,
                std::vector<cv::Point2f>& newkps,
                const cv::Mat& roi,
                const std::string& im_debug_path = std::string());

    void describeBRIEF(const cv::Mat& im,
                       const std::vector<cv::Point2f>& pts,
                       std::vector<cv::Mat>& descs);

  private:
    int _cell_size;
    double _max_quality;

    // just use it to calc BRIEF for cv::Point2f, no ROTATION or SCALE INVARIANCE
    cv::Ptr<cv::Feature2D> _brief;
};
}  // namespace vo_lab

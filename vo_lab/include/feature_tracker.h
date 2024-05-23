#pragma once
#include <opencv2/opencv.hpp>

namespace vo_lab {
class FeatureTracker {
  public:
    typedef std::shared_ptr<FeatureTracker> Ptr;
    FeatureTracker(int max_iter,
                   float max_px_precision,
                   float winsize,
                   float err,
                   float max_fbklt_dist,
                   int border)
        : _klt_convg_crit(
                  cv::TermCriteria::COUNT + cv::TermCriteria::EPS, max_iter, max_px_precision),
          _klt_winsize(cv::Size(winsize, winsize)),
          _klt_err(err),
          _max_fbklt_dist(max_fbklt_dist),
          _border(border){};

    /**
     * @brief set stereo undist map
     * @param vpts: input pts
     * @param vpriorkps: priror & ouput kps
     */
    void forwardBackwardKltTracking(const std::vector<cv::Mat>& prevpyr,
                                    const std::vector<cv::Mat>& nextpyr,
                                    int pyrlvl,
                                    std::vector<cv::Point2f>& vpts,
                                    std::vector<cv::Point2f>& vpriorkps,
                                    std::vector<bool>& vkpstatus) const;

    bool getLineMinSAD(const cv::Mat& iml,
                       const cv::Mat& imr,
                       const cv::Point2f& pt,
                       const uint nwinsize,
                       float& xprior,
                       float& l1err,
                       bool bgoleft) const;

  private:
    bool inBorder(const cv::Point2f& pt, const cv::Mat& im) const;

    cv::TermCriteria _klt_convg_crit;
    cv::Size _klt_winsize;
    float _klt_err;  // not useful
    float _max_fbklt_dist;
    int _border;
};
}  // namespace vo_lab
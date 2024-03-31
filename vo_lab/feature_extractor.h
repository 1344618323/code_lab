#pragma once
#include <opencv4/opencv2/opencv.hpp>

class FeatureExtractor {
  public:
    FeatureExtractor(int nfast_th = 10, double dmaxquality = 0.001);
    std::vector<cv::Point2f> detectGridFAST(const cv::Mat& im,
                                            const int ncellsize,
                                            const std::vector<cv::Point2f>& vcurkps,
                                            const cv::Mat& roi = cv::Mat());
    std::vector<cv::Point2f> detectSingleScale(const cv::Mat& im,
                                               const int ncellsize,
                                               const std::vector<cv::Point2f>& vcurkps,
                                               const cv::Mat& roi = cv::Mat());

  private:
    cv::Ptr<cv::FastFeatureDetector> pfast_;
    int nfast_th_;
    double dmaxquality_;
};

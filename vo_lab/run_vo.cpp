#include "feature_extractor.h"

std::string left_file = "/home/cxn/leofile/slambook2/ch5/stereo/left.png";
std::string right_file = "/home/cxn/leofile/slambook2/ch5/stereo/right.png";

int main(int argc, char** argv) {
    cv::Mat limg = cv::imread(left_file, cv::IMREAD_GRAYSCALE);
    cv::Mat rimg = cv::imread(right_file, cv::IMREAD_GRAYSCALE);
    // FeatureExtractor fe;
    // auto out = fe.detectSingleScale(limg, 35, std::vector<cv::Point2f>());
    // cv::Mat limg_c;
    // cv::cvtColor(limg, limg_c, cv::COLOR_BayerBG2BGR);
    // for (auto& e : out) {
    //     cv::circle(limg_c, e, 3, cv::Scalar(0, 255, 0), -1);
    // }
    // FeatureExtractor fe2;
    // out = fe2.detectSingleScale(rimg, 35, std::vector<cv::Point2f>());
    // cv::Mat rimg_c;
    // cv::cvtColor(rimg, rimg_c, cv::COLOR_BayerBG2BGR);
    // for (auto& e : out) {
    //     cv::circle(rimg_c, e, 3, cv::Scalar(0, 255, 0), -1);
    // }

    cv::Ptr<cv::GFTTDetector> gftt = cv::GFTTDetector::create(300, 0.01);
    std::vector<cv::KeyPoint> pt;
    gftt->detect(limg, pt);
    cv::Mat limg_c;
    cv::drawKeypoints(limg, pt, limg_c);
    pt.clear();
    gftt->detect(rimg, pt);
    cv::Mat rimg_c;
    cv::drawKeypoints(rimg, pt, rimg_c);
    cv::Mat cimg;
    cv::vconcat(limg_c, rimg_c, cimg);
    cv::imshow("show", cimg);
    cv::waitKey(0);
    return 0;
}
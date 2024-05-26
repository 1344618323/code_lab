#include <ceres/ceres.h>
// #include <pangolin/pangolin.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <unistd.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <condition_variable>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <shared_mutex>
#include <sophus/se3.hpp>
#include <thread>
#include <unordered_set>

std::string left_file = "/home/cxn/leofile/slam/slambook2/ch5/stereo/left.png";
std::string right_file = "/home/cxn/leofile/slam/slambook2/ch5/stereo/right.png";

using namespace std;

int main(int argc, char** argv) {
    std::vector<cv::Point2f> cvbv;
    std::vector<cv::Point3f> cvwpt;

    for (size_t i = 0; i < 4; i++) {
        cv::Mat m(2, 1, CV_32F);
        cv::randn(m, 0, 1);
        cvbv.emplace_back(m.at<float>(0, 0), m.at<float>(1, 0));
        cvwpt.emplace_back(m.at<float>(0, 0), m.at<float>(1, 0), 1);
    }
    cv::Mat D;
    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat tvec, rvec;
    cv::Mat inliers;
    cv::solvePnPRansac(
            cvwpt, cvbv, K, D, rvec, tvec, false, 100, 3 / 480.0, 0.99, inliers, cv::SOLVEPNP_P3P);
    if (inliers.rows == 0) {
        return false;
    }
    std::cout << inliers.depth() << " " << inliers.rows << " " << inliers.cols << std::endl;
    return 0;
}
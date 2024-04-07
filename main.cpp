#include <ceres/ceres.h>
// #include <pangolin/pangolin.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unistd.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

using namespace std;
using namespace Eigen;

// 文件路径
string left_file = "/home/cxn/leofile/slambook2/ch5/stereo/left.png";
string right_file = "/home/cxn/leofile/slambook2/ch5/stereo/right.png";

struct Base {
    void fun() { std::cout << "1" << std::endl; }
};

int main(int argc, char** argv) {
    cv::Mat m(100, 100, CV_8U, cv::Scalar(100));
    cv::imshow("cxn", m);
    cv::waitKey(0);
    return 0;
}
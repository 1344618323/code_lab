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
{
    // Eigen 是支持维度的隐式转换的
    Eigen::Matrix<double, 1, 3> ea;
    Eigen::Matrix<double, 3, 1> eb(1, 2, 3);
    ea = eb;  // (3,1)赋值到(1,3)
    std::cout << ea << std::endl;

    Eigen::Matrix3d em;
    em.row(1) = ea;
    em.col(2) = ea;
    em.row(0).head<2>() = ea.tail<2>();
    std::cout << em << std::endl;

    // opencv 就不支持啦: (3,1)赋值到(1,3) 会core
    cv::Mat cm(3, 3, CV_64F, cv::Scalar(0));
    cv::Mat ca = (cv::Mat_<double>(3, 1) << 1, 2, 3);
    cv::Mat cat;
    cv::transpose(ca, cat);
    cm.row(1) = cat;  // Wrong! cv.row(1) will return a new header, and this new header will
                        // point to cat
    cat.copyTo(cm.row(1));  // Right
    std::cout << cm << std::endl;
}

{
    Eigen::Matrix3d er = Eigen::Matrix3d::Random();
    cv::Mat cr(3, 3, CV_64F);
    cv::randu(cr, cv::Scalar(0), cv::Scalar(1));
    std::cout << er << "\n\n" << cr << std::endl;

    cv::Mat cer(3, 3, CV_32F);
    std::cout << "before " << (int*)(cer.data) << " " << cer.depth() << std::endl;
    cv::eigen2cv(er, cer);
    std::cout << "after " << (int*)(cer.data) << " " << cer.depth() << std::endl;
    /*
    before 0x55fe19850400 5
    after 0x55fe19850500 6
    可见 eigen2cv 时，cv::Mat 是另外开辟了一块内存
    */
}

{
    Eigen::Matrix2d em;
    cv::Mat cm = (cv::Mat_<float>(2, 2) << 1, 2, 3, 4);
    cv::cv2eigen(cm, em);
    std::cout << em
                << std::endl;  // cv2eigen： 会有float double的隐式转换，但没有维度的隐式转换
}

    return 0;
}
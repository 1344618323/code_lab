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
    return 0;
}
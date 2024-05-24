#include <ceres/ceres.h>
// #include <pangolin/pangolin.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <unistd.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <algorithm>
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

struct ListNode {
    int val;
    ListNode* next;
    ListNode() : val(0), next(nullptr) {}
    ListNode(int x) : val(x), next(nullptr) {}
    ListNode(int x, ListNode* next) : val(x), next(next) {}
};

struct TreeNode {
    int val;
    TreeNode* left;
    TreeNode* right;
    TreeNode() : val(0), left(nullptr), right(nullptr) {}
    TreeNode(int x) : val(x), left(nullptr), right(nullptr) {}
    TreeNode(int x, TreeNode* left, TreeNode* right) : val(x), left(left), right(right) {}
};

int main(int argc, char** argv) {
    vector<int> v = {2, 3, 4, 1};
    sort(v.begin(), v.end(), [](const int& a, const int& b) { return a > b; });
    for (auto& i : v) {
        std::cout << i << std::endl;
    }
};

#include <ceres/ceres.h>
#include <pangolin/pangolin.h>
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
    // ui shape
    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            // set camera matrix: width, height, fx, fy, cx, cy, show_min_z, show_max_z
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
            // set camera pos: cam tx ty tz; 相机看向点的位置 tx ty tz; 0.0, -1.0, 0.0 意味着
            // 相机坐标系的 -y 轴 朝向上方
            pangolin::ModelViewLookAt(0, -5.0, -10.0, 0, 0, 0, 0.0, -1.0, 0.0));
    // 这一通操作后，世界坐标系和相机坐标系是平行的：x轴向右，y轴向下，z轴向前

    pangolin::View& d_cam =
            // ui中用于显示图像区域的bound，这里的意思是整个ui都用于显示图像
            pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                    .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        // draw axis
        glLineWidth(3);
        glColor3f(0.8f, 0.f, 0.f);
        glBegin(GL_LINES);
        // x axis
        glVertex3f(0, 0, 0);
        glVertex3f(10, 0, 0);
        // y axis
        glColor3f(0.f, 0.8f, 0.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 10, 0);
        // z axis
        glColor3f(0.f, 0.f, 0.8f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 10);
        glEnd();

        // draw points
        glPointSize(2);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < 1000; i++) {
            glColor3f(0.8f, 0.8f, 0.8f);
            glVertex3d(i * 0.01, i * 0.01, i * 0.01);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);  // sleep 5 ms
    }
    return 0;
}
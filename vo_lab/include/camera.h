#pragma once
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

namespace vo_lab {
class Camera {
  public:
    typedef std::shared_ptr<Camera> Ptr;
    typedef std::shared_ptr<Camera const> ConstPtr;
    Camera(const cv::Mat& Kcv, const cv::Mat& Dcv, int width, int height);

    Sophus::SE3d Tc0ci() const { return _Tc0ci; }
    Sophus::SE3d Tcic0() const { return _Tcic0; }
    void setExtrinsic(const Sophus::SE3d& Tc0ci) {
        _Tc0ci = Tc0ci;
        _Tcic0 = Tc0ci.inverse();
    }
    void setMonoUndistMap(double alpha);

    /**
     * @brief set stereo undist map
     * @param R: R_{rectify origin}
     * @param P: input a 3*4 matrix.
     *        for left:
     *            f 0 cx 0
     *            0 f cy 0
     *            0 0  1 0
     *        for right:
     *            f 0 cx t_{rl}f
     *            0 f cy 0
     *            0 0  1 0
     * @param roi: rectangle inside the rectified image where all the pixels are valid
     * @return
     */
    void setStereoUndistMap(const cv::Mat& R, const cv::Mat& P, const cv::Rect& roi);
    void rectifyImage(const cv::Mat& src, cv::Mat& dst) const;
    cv::Rect roi_rect() const { return _roi_rect; }
    cv::Mat roi_mask() const { return _roi_mask; }
    bool isInImage(const cv::Point2f& px) const;
    cv::Point2f undistortPx(const cv::Point2f& px) const;
    cv::Point2f distortUnpx(const cv::Point2f& unpx) const;
    Eigen::Vector3d calcBearingVector(const cv::Point2f& unpx) const;
    cv::Point2f projCamToUnpx(const Eigen::Vector3d& pt) const;
    cv::Point2f projCamToPx(const Eigen::Vector3d& pt) const;
    Eigen::Matrix3d K() const { return _K; }
    Eigen::Matrix3d iK() const { return _iK; }
    cv::Size size() const { return cv::Size(_width, _height); }

  private:
    void setROIMask(const cv::Rect& rect);

    cv::Mat _Kcv;
    cv::Mat _Dcv;
    Eigen::Matrix3d _K, _iK;
    int _width, _height;

    Sophus::SE3d _Tc0ci;
    Sophus::SE3d _Tcic0;

    cv::Rect _roi_rect;
    cv::Mat _roi_mask;

    // undistort map
    cv::Mat _undist_map_x, _undist_map_y;
};
}  // namespace vo_lab

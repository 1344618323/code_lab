#include "camera.h"

namespace vo_lab {
Camera::Camera(const cv::Mat& Kcv, const cv::Mat& Dcv, int width, int height)
    : _Kcv(Kcv), _Dcv(Dcv), _width(width), _height(height) {
    cv::cv2eigen(_Kcv, _K);
    _iK = _K.inverse();
    setROIMask(cv::Rect(0, 0, _width, _height));
}

void Camera::setMonoUndistMap(double alpha) {
    cv::Size img_size(_width, _height);
    cv::Rect roi_rect;
    cv::Mat newK = cv::getOptimalNewCameraMatrix(_Kcv, _Dcv, img_size, alpha, img_size, &roi_rect);
    cv::initUndistortRectifyMap(
            _Kcv, _Dcv, cv::Mat(), newK, img_size, CV_32FC1, _undist_map_x, _undist_map_y);
    _Dcv.release();
    newK.copyTo(_Kcv);
    cv::cv2eigen(_Kcv, _K);
    _iK = _K.inverse();
    setROIMask(roi_rect);
}

void Camera::setStereoUndistMap(const cv::Mat& R, const cv::Mat& P, const cv::Rect& roi) {
    cv::Size img_size(_width, _height);
    cv::initUndistortRectifyMap(_Kcv, _Dcv, R, P, img_size, CV_32FC1, _undist_map_x, _undist_map_y);
    _Dcv.release();
    _Kcv = P(cv::Rect(0, 0, 3, 3));
    cv::cv2eigen(_Kcv, _K);
    _iK = _K.inverse();
    setROIMask(roi);
}

void Camera::rectifyImage(const cv::Mat& src, cv::Mat& dst) const {
    if (!_undist_map_x.empty()) {
        cv::remap(src, dst, _undist_map_x, _undist_map_y, cv::INTER_LINEAR);
    } else {
        dst = src;
    }
}

void Camera::setROIMask(const cv::Rect& rect) {
    const int border = 5;
    _roi_rect = rect;
    _roi_rect.width = std::max(1, _roi_rect.width - 2 * border);
    _roi_rect.height = std::max(1, _roi_rect.height - 2 * border);
    _roi_rect.x = std::min(_roi_rect.x + border, int(_width) - 1);
    _roi_rect.y = std::min(_roi_rect.y + border, int(_height) - 1);
    _roi_mask = cv::Mat(_height, _width, CV_8U, cv::Scalar(0));
    _roi_mask(_roi_rect).setTo(cv::Scalar(255));
}

bool Camera::isInImage(const cv::Point2f& px) const {
    return _roi_rect.contains(px);
}

cv::Point2f Camera::undistortPx(const cv::Point2f& px) const {
    if (_Dcv.empty()) {
        return px;
    }
    std::vector<cv::Point2f> vpx = {px};
    std::vector<cv::Point2f> vunpx;
    cv::undistortPoints(vpx, vunpx, _Kcv, _Dcv, _Kcv);
    return vunpx[0];
}

cv::Point2f Camera::distortUnpx(const cv::Point2f& unpx) const {
    return projCamToPx(calcBearingVector(unpx));
}

Eigen::Vector3d Camera::calcBearingVector(const cv::Point2f& unpx) const {
    Eigen::Vector3d hunpx(unpx.x, unpx.y, 1);
    Eigen::Vector3d bv = _iK * hunpx;
    bv.normalize();
    return bv;
}

cv::Point2f Camera::projCamToUnpx(const Eigen::Vector3d& pt) const {
    auto px = _K * pt;
    return cv::Point2f(px.x() / px.z(), px.y() / px.z());
}

cv::Point2f Camera::projCamToPx(const Eigen::Vector3d& pt) const {
    std::vector<cv::Point3f> vpt = {cv::Point3f(pt.x(), pt.y(), pt.z())};
    std::vector<cv::Point2f> vpx;
    cv::Mat rt = cv::Mat::zeros(3, 1, CV_32F);
    cv::projectPoints(vpt, rt, rt, _Kcv, _Dcv, vpx);
    return vpx[0];
}

}  // namespace vo_lab
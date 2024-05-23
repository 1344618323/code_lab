#pragma once
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include "camera.h"

namespace vo_lab {
struct Keypoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    uint64_t mpid;
    cv::Point2f px;
    cv::Point2f unpx;
    Eigen::Vector3d bv;  // bearing vector

    cv::Mat desc;

    bool is_stereo = false;
    cv::Point2f rpx;
    cv::Point2f runpx;
    Eigen::Vector3d rbv;

    // It should be noted that `is_stereo` indicate whether there's stereo matching
    // point in this frame. `is_3d` may come from historical frames. There may be situations
    // where `is_3d=true` and `is_stereo=false`
    bool is_3d = false;
};

class Frame {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Frame> Ptr;
    typedef std::shared_ptr<Frame const> ConstPtr;

    Frame(uint64_t id_,
          double time_,
          const cv::Mat& im_left_,
          const cv::Mat& im_right_,
          const Camera::ConstPtr cam_left_,
          const Camera::ConstPtr cam_right_,
          int cellsize);
    // Frame(const Frame& F);

    bool addKeypoint(uint64_t mpid, const cv::Point2f& px, const cv::Mat& desc, bool is_3d = false);
    void removeKeypoint(uint64_t mpid);
    bool updateKeypointStereo(uint64_t mpid, const cv::Point2f& rpx);
    void removeKeypointStereo(uint64_t);
    bool turnKeypoint3d(uint64_t mpid);
    void getKeypoints(std::vector<Keypoint>& v) const;
    void getKeypointsStereo(std::vector<Keypoint>& v) const;
    void getKeypoints3d(std::vector<Keypoint>& v) const;
    void getKeppointIdsGrid(std::vector<std::set<uint64_t>>& kpid_grid, size_t& nbkps);
    bool getKeypointById(uint64_t mpid, Keypoint& kp);
    Sophus::SE3d Twc() const;
    Sophus::SE3d Tcw() const;
    void setTwc(const Sophus::SE3d& T);
    Camera::ConstPtr camLeft() const { return _cam_left; }
    Camera::ConstPtr camRight() const { return _cam_right; }
    Eigen::Matrix3d Frl() const { return _Frl; }
    Eigen::Vector3d projWorldToCam(const Eigen::Vector3d& wpt) const;
    cv::Point2f projWorldToUnpx(const Eigen::Vector3d& wpt) const;
    cv::Point2f projWorldToPx(const Eigen::Vector3d& wpt) const;
    cv::Point2f projWorldToRightUnpx(const Eigen::Vector3d& wpt) const;
    cv::Point2f projWorldToRightPx(const Eigen::Vector3d& wpt) const;
    bool isInImage(const cv::Point2f& px) const;
    bool isInRightImage(const cv::Point2f& px) const;

    const uint64_t id;
    const double time;
    const cv::Mat im_left_raw;
    const cv::Mat im_right_raw;
    cv::Mat im_left;
    cv::Mat im_right;
    std::vector<cv::Mat> pyr_left;
    std::vector<cv::Mat> pyr_right;

    struct KpsGrid {
        int cellsize;
        int wcells;
        int hcells;
        size_t nbkps = 0;
        std::unordered_map<uint64_t, Keypoint> map_kps;
        std::vector<std::set<uint64_t>> grid;
        bool addKpToGrid(const Keypoint& kp);
        void removeKpFromGrid(uint64_t mpid);

      private:
        int getKpCellIdx(const cv::Point2f& px) const;
    };

  private:
    Camera::ConstPtr _cam_left;
    Camera::ConstPtr _cam_right;
    Eigen::Matrix3d _Frl;

    mutable std::mutex _pose_mutex;
    Sophus::SE3d _Twc, _Tcw;
    mutable std::mutex _kps_mutex;
    KpsGrid _kps_grid;
};
}  // namespace vo_lab

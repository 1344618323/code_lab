#include "frame.h"
#include "mappoint.h"
namespace vo_lab {

bool Keypoint::is3D() const {
    auto mpptr = mp.lock();
    if (mpptr) {
        return mpptr->is3D();
    }
    return false;
}

Frame::Frame(uint64_t id_,
             double time_,
             const cv::Mat& im_left_,
             const cv::Mat& im_right_,
             const Camera::ConstPtr cam_left_,
             const Camera::ConstPtr cam_right_,
             int cellsize)
    : id(id_),
      time(time_),
      im_left_raw(im_left_),
      im_right_raw(im_right_),
      im_left(im_left_),
      im_right(im_right_),
      _cam_left(cam_left_),
      _cam_right(cam_right_) {
    if (_cam_right) {
        Eigen::Vector3d trl = _cam_right->Tcic0().translation();
        Eigen::Matrix3d Erl = Sophus::SO3d::hat(trl) * _cam_right->Tcic0().rotationMatrix();
        _Frl = _cam_right->iK().transpose() * Erl * _cam_left->iK();
    }

    _kps_grid.cellsize = cellsize;
    _kps_grid.wcells = std::ceil(float(_cam_left->size().width) / cellsize);
    _kps_grid.hcells = std::ceil(float(_cam_left->size().height) / cellsize);
    _kps_grid.grid.resize(_kps_grid.wcells * _kps_grid.hcells);
    _kps_grid.nbkps = 0;
}

// Frame::Frame(const Frame& F)
//     : id(F.id),
//       time(F.time),
//       im_left_raw(F.im_left),
//       im_right_raw(F.im_right),
//       im_left(F.im_left),
//       im_right(F.im_right),
//       pyr_left(F.pyr_left),
//       pyr_right(F.pyr_right),
//       _cam_left(F._cam_left),
//       _cam_right(F._cam_right),
//       _Frl(F._Frl),
//       _Twc(F._Twc),
//       _Tcw(F._Tcw),
//       _map_kps(F._map_kps) {}

bool Frame::addKeypoint(const std::shared_ptr<Mappoint>& mp,
                        const cv::Point2f& px,
                        const cv::Mat& desc) {
    if (!mp) {
        return false;
    }
    Keypoint kp;
    kp.mpid = mp->mpid;
    kp.px = px;
    kp.unpx = _cam_left->undistortPx(kp.px);
    kp.bv = _cam_left->calcBearingVector(kp.unpx);
    kp.desc = desc;
    kp.mp = mp;
    std::lock_guard<std::mutex> lock(_kps_mutex);
    return _kps_grid.addKpToGrid(kp);
}

void Frame::removeKeypoint(uint64_t mpid) {
    std::lock_guard<std::mutex> lock(_kps_mutex);
    _kps_grid.removeKpFromGrid(mpid);
}

void Frame::clearKeypoints() {
    std::lock_guard<std::mutex> lock(_kps_mutex);
    _kps_grid.clear();
}

bool Frame::updateKeypointStereo(uint64_t mpid, const cv::Point2f& rpx) {
    std::lock_guard<std::mutex> lock(_kps_mutex);
    auto it = _kps_grid.map_kps.find(mpid);
    if (it == _kps_grid.map_kps.end()) {
        return false;
    }
    auto& kp = it->second;
    kp.rpx = rpx;
    kp.runpx = _cam_right->undistortPx(kp.rpx);
    kp.rbv = _cam_right->calcBearingVector(kp.runpx);
    if (!kp.is_stereo) {
        kp.is_stereo = true;
    }
    return true;
}

void Frame::removeKeypointStereo(uint64_t mpid) {
    std::lock_guard<std::mutex> lock(_kps_mutex);
    auto it = _kps_grid.map_kps.find(mpid);
    if (it == _kps_grid.map_kps.end()) {
        return;
    }
    auto& kp = it->second;
    if (kp.is_stereo) {
        kp.is_stereo = false;
    }
}

void Frame::getKeypoints(std::vector<Keypoint>& v) const {
    v.clear();
    std::lock_guard<std::mutex> lock(_kps_mutex);
    for (auto& kpit : _kps_grid.map_kps) {
        v.push_back(kpit.second);
    }
}

void Frame::getKeypointsStereo(std::vector<Keypoint>& v) const {
    v.clear();
    std::lock_guard<std::mutex> lock(_kps_mutex);
    for (auto& kpit : _kps_grid.map_kps) {
        if (kpit.second.is_stereo) {
            v.push_back(kpit.second);
        }
    }
}

void Frame::getKeypoints3d(std::vector<Keypoint>& v) const {
    v.clear();
    std::lock_guard<std::mutex> lock(_kps_mutex);
    for (auto& kpit : _kps_grid.map_kps) {
        if (kpit.second.is3D()) {
            v.push_back(kpit.second);
        }
    }
}

void Frame::getKeppointIdsGrid(std::vector<std::set<uint64_t>>& kpid_grid, size_t& nbkps) const {
    kpid_grid.clear();
    std::lock_guard<std::mutex> lock(_kps_mutex);
    kpid_grid = _kps_grid.grid;
    nbkps = _kps_grid.nbkps;
}

bool Frame::getKeypoint(uint64_t mpid, Keypoint& kp) const {
    std::lock_guard<std::mutex> lock(_kps_mutex);
    auto it = _kps_grid.map_kps.find(mpid);
    if (it == _kps_grid.map_kps.end()) {
        return false;
    }
    kp = it->second;
    return true;
}

Sophus::SE3d Frame::Twc() const {
    std::lock_guard<std::mutex> lock(_pose_mutex);
    return _Twc;
}

Sophus::SE3d Frame::Tcw() const {
    std::lock_guard<std::mutex> lock(_pose_mutex);
    return _Tcw;
}

void Frame::setTwc(const Sophus::SE3d& T) {
    std::lock_guard<std::mutex> lock(_pose_mutex);
    _Twc = T;
    _Tcw = T.inverse();
}

Eigen::Vector3d Frame::projWorldToCam(const Eigen::Vector3d& wpt) const {
    std::lock_guard<std::mutex> lock(_pose_mutex);
    return _Tcw * wpt;
}

cv::Point2f Frame::projWorldToUnpx(const Eigen::Vector3d& wpt) const {
    return _cam_left->projCamToUnpx(projWorldToCam(wpt));
}

cv::Point2f Frame::projWorldToPx(const Eigen::Vector3d& wpt) const {
    return _cam_left->projCamToPx(projWorldToCam(wpt));
}

cv::Point2f Frame::projWorldToRightUnpx(const Eigen::Vector3d& wpt) const {
    return _cam_right->projCamToUnpx(_cam_right->Tcic0() * projWorldToCam(wpt));
}

cv::Point2f Frame::projWorldToRightPx(const Eigen::Vector3d& wpt) const {
    return _cam_right->projCamToPx(_cam_right->Tcic0() * projWorldToCam(wpt));
}

bool Frame::isInImage(const cv::Point2f& px) const {
    return _cam_left->isInImage(px);
}

bool Frame::isInRightImage(const cv::Point2f& px) const {
    return _cam_right->isInImage(px);
}

bool Frame::KpsGrid::addKpToGrid(const Keypoint& kp) {
    int idx = getKpCellIdx(kp.px);
    if (idx < 0) {
        return false;
    }
    grid[idx].insert(kp.mpid);
    map_kps.emplace(kp.mpid, kp);
    nbkps++;
    return true;
}

void Frame::KpsGrid::removeKpFromGrid(uint64_t mpid) {
    auto it = map_kps.find(mpid);
    if (it != map_kps.end()) {
        int idx = getKpCellIdx(it->second.px);
        if (idx >= 0) {
            grid[idx].erase(mpid);
        }
        map_kps.erase(it);
        nbkps--;
    }
}

void Frame::KpsGrid::clear() {
    map_kps.clear();
    for (auto& cell : grid) {
        cell.clear();
    }
    nbkps = 0;
}

int Frame::KpsGrid::getKpCellIdx(const cv::Point2f& px) const {
    int r = std::floor(px.y / cellsize);
    int c = std::floor(px.x / cellsize);
    if (r < 0 || r >= hcells || c < 0 || c >= wcells) {
        return -1;
    }
    return r * wcells + c;
}
}  // namespace vo_lab
// namespace vo_lab

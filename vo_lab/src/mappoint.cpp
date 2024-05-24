#include "mappoint.h"

namespace vo_lab {
Mappoint::Mappoint(const uint64_t mpid_, const Frame::Ptr anch_kf, const cv::Mat& desc)
    : mpid(mpid_), _anch_kf(anch_kf) {
    _kfs.emplace(anch_kf->id, anch_kf);
    if (!desc.empty()) {
        _map_kf_desc.emplace(anch_kf, desc);
    }
}

void Mappoint::setPos(const Eigen::Vector3d& pos) {
    std::lock_guard<std::mutex> lk(_mp_mutex);
    _is_3d = true;
    _pos = pos;
}

bool Mappoint::getPos(Eigen::Vector3d& pos) const {
    std::lock_guard<std::mutex> lk(_mp_mutex);
    if (!_is_3d) {
        return false;
    }
    pos = _pos;
    return true;
}

bool Mappoint::is3D() const {
    std::lock_guard<std::mutex> lk(_mp_mutex);
    return _is_3d;
}

void Mappoint::addKfObs(const Frame::Ptr kf, const cv::Mat& desc) {
    std::lock_guard<std::mutex> lk(_mp_mutex);
    _kfs.emplace(kf->id, kf);
    if (!desc.empty()) {
        _map_kf_desc.emplace(kf, desc);
    }
}

void Mappoint::removeKfObs(const Frame::Ptr kf) {
    std::lock_guard<std::mutex> lk(_mp_mutex);
    _kfs.erase(kf->id);
    _map_kf_desc.erase(kf);
}

std::map<uint64_t, Frame::Ptr> Mappoint::getKfObs() const {
    std::lock_guard<std::mutex> lk(_mp_mutex);
    return _kfs;
}

}  // namespace vo_lab

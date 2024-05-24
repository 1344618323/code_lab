#include "map.h"

namespace vo_lab {

Map::Map(const Config& config) : _config(config) {}

void Map::addKeyframe(const Frame::Ptr kf) {
    std::lock_guard<std::mutex> lock(_kf_mutex);
    _map_kfs.emplace(kf->id, kf);
}

void Map::addMappoint(const Mappoint::Ptr mp) {
    std::lock_guard<std::mutex> lock(_mp_mutex);
    _map_mpts.emplace(mp->mpid, mp);
}

Mappoint::Ptr Map::getMappoint(uint64_t mpid) const {
    std::lock_guard<std::mutex> lock(_mp_mutex);
    auto mpit = _map_mpts.find(mpid);
    return mpit == _map_mpts.end() ? nullptr : mpit->second;
}

std::map<uint64_t, Frame::Ptr> Map::kfs() const {
    std::lock_guard<std::mutex> lock(_kf_mutex);
    return _map_kfs;
}
}  // namespace vo_lab

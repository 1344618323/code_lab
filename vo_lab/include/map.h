#pragma once

#include "config.h"
#include "feature_extractor.h"
#include "feature_tracker.h"
#include "frame.h"
#include "mappoint.h"

namespace vo_lab {
class Map {
  public:
    typedef std::shared_ptr<Map> Ptr;
    typedef std::shared_ptr<Map const> ConstPtr;
    Map(const Config& config);
    void addKeyframe(const Frame::Ptr kf);
    void addMappoint(const Mappoint::Ptr mp);
    Mappoint::Ptr getMappoint(uint64_t mpid) const;

  private:
    const Config& _config;
    mutable std::mutex _kf_mutex, _mp_mutex;
    std::map<uint64_t, Frame::Ptr> _map_kfs;
    std::unordered_map<uint64_t, Mappoint::Ptr> _map_mpts;
};
}  // namespace vo_lab

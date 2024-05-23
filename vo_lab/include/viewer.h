#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <condition_variable>
#include <thread>

#include "frame.h"
#include "map.h"

namespace vo_lab {
class Viewer {
  public:
    typedef std::shared_ptr<Viewer> Ptr;
    Viewer(const Map::ConstPtr map);
    ~Viewer();
    void addFrame(const Frame::ConstPtr frame);

  private:
    void viewThread();

    const Map::ConstPtr _map;
    std::thread _view_thread;
    std::atomic<bool> _stop = false;
    pcl::visualization::PCLVisualizer::Ptr _pcl_viewer = nullptr;

    std::condition_variable _frames_cv;
    std::mutex _frames_mutex;
    std::queue<Frame::ConstPtr> _frames;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _local_mps = nullptr;
};
}  // namespace vo_lab
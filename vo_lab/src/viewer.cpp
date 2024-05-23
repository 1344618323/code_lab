#include "viewer.h"

namespace vo_lab {
Viewer::Viewer(const Map::ConstPtr map) : _map(map) {
    _pcl_viewer.reset(new pcl::visualization::PCLVisualizer());
    _local_mps.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    _view_thread = std::thread(&Viewer::viewThread, this);
}

Viewer::~Viewer() {
    _stop = true;
    _frames_cv.notify_all();
    _view_thread.join();
}

void Viewer::addFrame(const Frame::ConstPtr frame) {
    std::lock_guard<std::mutex> lk(_frames_mutex);
    _frames.push(frame);
    _frames_cv.notify_one();
}

void Viewer::viewThread() {
    _pcl_viewer->addCoordinateSystem(10, "odom");
    while (!_stop) {
        std::shared_ptr<Frame const> frame;
        {
            std::unique_lock<std::mutex> lk(_frames_mutex);
            _frames_cv.wait(lk, [this]() { return _stop || !_frames.empty(); });
            if (_stop) {
                return;
            }
            frame = _frames.front();
            _frames.pop();
        }
        _pcl_viewer->removeCoordinateSystem("ego");
        _pcl_viewer->removePointCloud("local_mps");
        Eigen::Affine3f T;
        T.matrix() = frame->Twc().matrix().cast<float>();
        _pcl_viewer->addCoordinateSystem(10, T, "ego");

        _local_mps->clear();
        std::vector<Keypoint> kps3d;
        frame->getKeypoints3d(kps3d);
        for (size_t i = 0; i < kps3d.size(); i++) {
            auto mp = _map->getMappoint(kps3d.at(i).mpid);
            Eigen::Vector3d mp_pos;
            if (!mp || !mp->getPos(mp_pos)) {
                continue;
            }
            pcl::PointXYZRGB pt(0, 255, 0);
            pt.x = mp_pos.x();
            pt.y = mp_pos.y();
            pt.z = mp_pos.z();
            _local_mps->push_back(pt);
        }
        _pcl_viewer->addPointCloud(_local_mps, "local_mps");
        _pcl_viewer->spinOnce(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
}  // namespace vo_lab
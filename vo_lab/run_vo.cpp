#include <thread>

#include <glog/logging.h>

#include <cv_bridge/cv_bridge.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>

#include "vo.h"

// usage: ./build/latest/vo_lab/run_vo

DEFINE_string(config, "/home/cxn/leofile/code_lab/vo_lab/config/config.yaml", "config yaml");

class SensorsGrabber {
  public:
    SensorsGrabber(const vo_lab::Config& config) : _config(config) {}

    void subLeftImage(const sensor_msgs::ImageConstPtr& img) {
        std::lock_guard<std::mutex> lock(_img_mutex);
        _im_left_buf.push(img);
    }
    void subRightImage(const sensor_msgs::ImageConstPtr& img) {
        std::lock_guard<std::mutex> lock(_img_mutex);
        _im_right_buf.push(img);
    }
    void subImu(const sensor_msgs::ImuConstPtr& imu) {
        std::lock_guard<std::mutex> lock(_imu_mutex);
        _imu_buf.push(imu);
    }

    void stop() { _stop = true; };

    void sync_process() {
        vo_lab::VO vo(_config);
        bool im_buf_empty = false;
        while (!(_stop && im_buf_empty)) {
            cv::Mat im_left, imright;
            double t_left = 0, t_right = 0;
            bool get_im = false;

            auto getGrayImg = [](const sensor_msgs::ImageConstPtr& img) {
                cv_bridge::CvImageConstPtr ptr =
                        cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
                return ptr->image;
            };

            {
                std::lock_guard<std::mutex> lock(_img_mutex);
                im_buf_empty = _im_left_buf.empty();
                if (_config.stereo && !_im_left_buf.empty() && !_im_right_buf.empty()) {
                    t_left = _im_left_buf.front()->header.stamp.toSec();
                    t_right = _im_right_buf.front()->header.stamp.toSec();
                    // sync tolerance
                    const double stereo_sync_tolerance = 0.01;
                    if (t_left < t_right - stereo_sync_tolerance) {
                        _im_left_buf.pop();
                    } else if (t_left > t_right + stereo_sync_tolerance) {
                        _im_right_buf.pop();
                    } else {
                        im_left = getGrayImg(_im_left_buf.front());
                        imright = getGrayImg(_im_right_buf.front());
                        _im_left_buf.pop();
                        _im_right_buf.pop();
                        get_im = true;
                    }
                } else if (!_config.stereo && !_im_left_buf.empty()) {
                    t_left = _im_left_buf.front()->header.stamp.toSec();
                    im_left = getGrayImg(_im_left_buf.front());
                    _im_left_buf.pop();
                    get_im = true;
                }
            }
            if (get_im) {
                if (_config.imu) {
                    std::lock_guard<std::mutex> lock(_imu_mutex);
                    if (!_imu_buf.empty()) {
                    }
                }
                vo.addImage(t_left, im_left, imright);
            }
        }
    }

  private:
    const vo_lab::Config& _config;
    std::mutex _img_mutex;
    std::queue<sensor_msgs::ImageConstPtr> _im_left_buf;
    std::queue<sensor_msgs::ImageConstPtr> _im_right_buf;
    std::mutex _imu_mutex;
    std::queue<sensor_msgs::ImuConstPtr> _imu_buf;
    std::atomic<bool> _stop = false;
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;

    const cv::FileStorage fs(FLAGS_config, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        LOG(ERROR) << "Failed to open configs file...";
        return 0;
    }
    vo_lab::Config config(fs);
    FLAGS_log_dir = config.debugger.dir;

    LOG(INFO) << "run ros bag " << config.rosbag_file;
    rosbag::Bag rosbag(config.rosbag_file, rosbag::bagmode::Read);
    if (!rosbag.isOpen()) {
        LOG(ERROR) << "cannot open " << config.rosbag_file;
        return 0;
    }

    SensorsGrabber grabber(config);
    std::thread sync_thread(&SensorsGrabber::sync_process, &grabber);
    rosbag::View view(rosbag);
    for (const rosbag::MessageInstance& m : view) {
        if (m.getTopic() == config.im_left_topic) {
            auto msg = m.instantiate<sensor_msgs::Image>();
            grabber.subLeftImage(msg);
        } else if (config.stereo && m.getTopic() == config.im_right_topic) {
            auto msg = m.instantiate<sensor_msgs::Image>();
            grabber.subRightImage(msg);
        } else if (config.imu && m.getTopic() == config.imu_topic) {
            auto msg = m.instantiate<sensor_msgs::Imu>();
            grabber.subImu(msg);
        }
    }
    rosbag.close();
    grabber.stop();
    sync_thread.join();
    return 0;
}

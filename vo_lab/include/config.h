#pragma once
#include <opencv2/opencv.hpp>

namespace vo_lab {

struct Debugger {
    std::string dir;
    bool debug;
    std::string im_input;
    std::string kps_extract;
    std::string klt_tracking;
    std::string stereo_matching;
};

struct Config {
    Config(const cv::FileStorage& fs);

    Debugger debugger;

    std::string rosbag_file;
    bool stereo;
    bool imu;
    std::string im_left_topic;
    std::string im_right_topic;
    std::string imu_topic;
    bool stereo_rect;
    bool mono_undist;

    // camera calibration
    int width, height;
    cv::Mat Kl, Kr;
    cv::Mat Dl, Dr;
    cv::Mat T_b_cl, T_b_cr;
    cv::Mat T_cl_cr;
    double undist_alpha;

    // image processing
    bool use_clahe;
    double clahe_val;
    int clahe_tile_size;

    // keypoints extraction
    int cellsize;
    double max_quality;

    // klt tracker
    int klt_winsize;
    int klt_pyr_lvl;
    bool klt_use_prior;
    int klt_pyr_lvl_prior;
    int max_iter;
    float max_px_precision;
    float klt_err;
    float max_fbklt_dist;
    int klt_border;

    // stereo matching
    float stereo_match_epi_err;       // 2
    int stereo_match_sad_winsize;     // 7
    int stereo_match_3d_klt_pyr_lvl;  // 1

    // reproj
    float max_reproj_dist;

    // epi
    int epi_ransac_iter;
    float epi_errth;
};
}  // namespace vo_lab

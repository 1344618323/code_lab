#include "config.h"
#include "utils.h"

namespace vo_lab {

Config::Config(const cv::FileStorage& fs) {
    // debugger
    cv::FileNode debugger_node = fs["debugger"];
    debugger_node["dir"] >> debugger.dir;
    mkdir(debugger.dir);
    debugger_node["debug"] >> debugger.debug;
    if (debugger.debug) {
        debugger_node["im_input"] >> debugger.im_input;
        debugger.im_input = debugger.dir + "/" + debugger.im_input;
        mkdir(debugger.im_input);
        debugger_node["kps_extract"] >> debugger.kps_extract;
        debugger.kps_extract = debugger.dir + "/" + debugger.kps_extract;
        mkdir(debugger.kps_extract);
        debugger_node["klt_tracking"] >> debugger.klt_tracking;
        debugger.klt_tracking = debugger.dir + "/" + debugger.klt_tracking;
        mkdir(debugger.klt_tracking);
        debugger_node["stereo_matching"] >> debugger.stereo_matching;
        debugger.stereo_matching = debugger.dir + "/" + debugger.stereo_matching;
        mkdir(debugger.stereo_matching);
    }

    fs["rosbag_file"] >> rosbag_file;
    fs["im_left_topic"] >> im_left_topic;
    fs["mono_undist"] >> mono_undist;
    fs["undist_alpha"] >> undist_alpha;

    fs["width"] >> width;
    fs["height"] >> height;
    fs["Kl"] >> Kl;
    fs["Dl"] >> Dl;
    fs["T_b_cl"] >> T_b_cl;

    fs["stereo"] >> stereo;
    if (stereo) {
        fs["stereo_rect"] >> stereo_rect;
        fs["im_right_topic"] >> im_right_topic;
        fs["Kr"] >> Kr;
        fs["Dr"] >> Dr;
        fs["T_b_cr"] >> T_b_cr;
        cv::Mat T_cl_b;
        cv::invert(T_b_cl, T_cl_b);
        T_cl_cr = T_cl_b * T_b_cr;
    }

    // imu
    fs["imu"] >> imu;
    if (imu) {
        fs["imu_topic"] >> imu_topic;
    }

    // clahe
    fs["use_clahe"] >> use_clahe;
    fs["clahe_val"] >> clahe_val;
    fs["clahe_tile_size"] >> clahe_tile_size;

    // feature extractor
    fs["cell_size"] >> cellsize;
    fs["max_quality"] >> max_quality;

    // klt tracker
    fs["klt_winsize"] >> klt_winsize;
    fs["klt_pyr_lvl"] >> klt_pyr_lvl;
    fs["klt_use_prior"] >> klt_use_prior;
    fs["klt_pyr_lvl_prior"] >> klt_pyr_lvl_prior;
    fs["max_iter"] >> max_iter;
    fs["max_px_precision"] >> max_px_precision;
    fs["klt_err"] >> klt_err;
    fs["max_fbklt_dist"] >> max_fbklt_dist;
    fs["klt_border"] >> klt_border;

    // stereo matching
    fs["stereo_match_epi_err"] >> stereo_match_epi_err;
    fs["stereo_match_sad_winsize"] >> stereo_match_sad_winsize;
    fs["stereo_match_3d_klt_pyr_lvl"] >> stereo_match_3d_klt_pyr_lvl;

    // reproj
    fs["max_reproj_dist"] >> max_reproj_dist;

    // epi
    fs["epi_ransac_iter"] >> epi_ransac_iter;
    fs["epi_errth"] >> epi_errth;
}

}  // namespace vo_lab
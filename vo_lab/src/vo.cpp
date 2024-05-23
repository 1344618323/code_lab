#include "vo.h"

namespace vo_lab {

VO::VO(const Config& config) : _config(config) {
    // load calibration
    _cam_left = std::make_shared<Camera>(_config.Kl, _config.Dl, _config.width, _config.height);
    if (_config.stereo) {
        _cam_right =
                std::make_shared<Camera>(_config.Kr, _config.Dr, _config.width, _config.height);
        Eigen::Matrix3d Rlr;
        Eigen::Vector3d tlr;
        cv::cv2eigen(_config.T_cl_cr(cv::Rect(0, 0, 3, 3)), Rlr);
        cv::cv2eigen(_config.T_cl_cr(cv::Rect(3, 0, 1, 3)), tlr);
        _cam_right->setExtrinsic(Sophus::SE3d(Rlr, tlr));
    }
    if (_config.stereo && _config.stereo_rect) {
        cv::Mat Tcv_cic0, Rcv_cic0, tcv_cic0;
        cv::invert(_config.T_cl_cr, Tcv_cic0);
        Rcv_cic0 = Tcv_cic0(cv::Rect(0, 0, 3, 3));
        tcv_cic0 = Tcv_cic0(cv::Rect(3, 0, 1, 3));
        cv::Mat Rl, Rr, Pl, Pr, Q;
        cv::Rect rect_left, rect_right;
        cv::Size img_size(_config.width, _config.height);
        cv::stereoRectify(_config.Kl,
                          _config.Dl,
                          _config.Kr,
                          _config.Dr,
                          img_size,
                          Rcv_cic0,
                          tcv_cic0,
                          Rl,
                          Rr,
                          Pl,
                          Pr,
                          Q,
                          cv::CALIB_ZERO_DISPARITY,
                          _config.undist_alpha,
                          img_size,
                          &rect_left,
                          &rect_right);
        _cam_left->setStereoUndistMap(Rl, Pl, rect_left);
        _cam_right->setStereoUndistMap(Rr, Pr, rect_right);
    } else if (_config.mono_undist) {
        _cam_left->setMonoUndistMap(_config.undist_alpha);
        if (_config.stereo) {
            _cam_right->setMonoUndistMap(_config.undist_alpha);
        }
    }
    _clahe = cv::createCLAHE(_config.clahe_val,
                             cv::Size(_config.width / _config.clahe_tile_size,
                                      _config.height / _config.clahe_tile_size));
    _extractor = std::make_shared<FeatureExtractor>(_config.cellsize, _config.max_quality);
    _tracker = std::make_shared<FeatureTracker>(_config.max_iter,
                                                _config.max_px_precision,
                                                _config.klt_winsize,
                                                _config.klt_err,
                                                _config.max_fbklt_dist,
                                                _config.klt_border);
    _map = std::make_shared<Map>(_config);
    _viewer = std::make_shared<Viewer>(_map);
    _frontend = std::make_shared<Frontend>(_config, _map, _clahe, _extractor, _tracker);
    _backend = std::make_shared<Backend>(_config, _map, _clahe, _tracker);
}

VO::~VO() {}

void VO::addImage(double time, cv::Mat& im_left, cv::Mat& im_right) {
    _cam_left->rectifyImage(im_left, im_left);
    if (_config.stereo) {
        _cam_right->rectifyImage(im_right, im_right);
    }
    if (_config.debugger.debug) {
        std::string img_path = _config.debugger.im_input + "/" + std::to_string(time) + ".png";
        cv::Mat img = im_left;
        if (_config.stereo) {
            cv::hconcat(im_left, im_right, img);
        }
        cv::imwrite(img_path, img);
    }

    std::shared_ptr<Frame> curframe = std::make_shared<Frame>(
            _frame_id, time, im_left, im_right, _cam_left, _cam_right, _config.cellsize);
    _frame_id++;

    bool is_kf = _frontend->tracking(curframe);
    if (is_kf) {
        _backend->addKeyframe(curframe);
    }
    _viewer->addFrame(curframe);
}
}  // namespace vo_lab

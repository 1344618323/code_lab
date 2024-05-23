#include "backend.h"
#include <glog/logging.h>
#include <unordered_set>
#include "multi_view_geometry.h"

namespace vo_lab {
Backend::Backend(const Config& config,
                 std::shared_ptr<Map> map,
                 const cv::Ptr<cv::CLAHE> clahe,
                 const FeatureTracker::Ptr tracker)
    : _config(config), _map(map), _clahe(clahe), _tracker(tracker) {
    _mapping_thread = std::thread(&Backend::mapping, this);
}

Backend::~Backend() {
    _stop = true;
    _kf_cv.notify_one();
    _mapping_thread.join();
}

void Backend::addKeyframe(const Frame::Ptr kf) {
    std::lock_guard<std::mutex> lk(_kf_mutex);
    _kfs.push(kf);
    _kf_cv.notify_one();
}

void Backend::mapping() {
    while (!_stop) {
        std::shared_ptr<Frame> kf;
        {
            std::unique_lock<std::mutex> lk(_kf_mutex);
            _kf_cv.wait(lk, [this]() { return _stop || !_kfs.empty(); });
            if (_stop) {
                return;
            }
            kf = _kfs.front();
            _kfs.pop();
        }
        if (_config.stereo) {
            _clahe->apply(kf->im_right_raw, kf->im_right);
            cv::buildOpticalFlowPyramid(kf->im_right,
                                        kf->pyr_right,
                                        cv::Size(_config.klt_winsize, _config.klt_winsize),
                                        _config.klt_pyr_lvl);
            stereoMatching(kf);
            triangulateStereo(kf);
        }
    }
}

void Backend::stereoMatching(const Frame::Ptr kf) {
    std::vector<Keypoint> vleftkps;
    kf->getKeypoints(vleftkps);
    size_t nbkps = vleftkps.size();

    std::vector<uint64_t> v3dkpids, vkpids;
    std::vector<cv::Point2f> v3dkps, v3dpriors, vkps, vpriors;
    v3dkpids.reserve(nbkps);
    vkpids.reserve(nbkps);
    v3dkps.reserve(nbkps);
    v3dpriors.reserve(nbkps);
    vkps.reserve(nbkps);
    vpriors.reserve(nbkps);

    // step1: calc prior for stereo tracking
    for (size_t i = 0; i < nbkps; i++) {
        auto& kp = vleftkps[i];

        // 1. calc 3d prior
        if (kp.is_3d) {
            auto mp = _map->getMappoint(kp.mpid);
            if (mp) {
                Eigen::Vector3d wpt;
                if (mp->getPos(wpt)) {
                    cv::Point2f rpx = kf->projWorldToRightPx(wpt);
                    if (kf->isInRightImage(rpx)) {
                        v3dkps.push_back(kp.px);
                        v3dpriors.push_back(rpx);
                        v3dkpids.push_back(kp.mpid);
                        continue;
                    }
                }
            } else {
                kf->removeKeypoint(kp.mpid);
                continue;
            }
        }

        // 2. calc 2d prior
        cv::Point2f priorpx = kp.px;
        if (_config.stereo_rect) {
            // 2.1 rectify:
            size_t maxpyrlvl = _config.klt_pyr_lvl * 2;
            auto winsize = _config.stereo_match_sad_winsize;
            float uppyrcoef = std::pow(2, _config.klt_pyr_lvl);
            float downpyrcoef = 1. / uppyrcoef;
            cv::Point2f pyrleftpx = kp.px * downpyrcoef;
            float xprior;
            float l1err;
            if (_tracker->getLineMinSAD(kf->pyr_left.at(maxpyrlvl),
                                        kf->pyr_right.at(maxpyrlvl),
                                        pyrleftpx,
                                        winsize,
                                        xprior,
                                        l1err,
                                        true)) {
                xprior *= uppyrcoef;
                if (xprior >= 0 && xprior <= kp.px.x) {
                    priorpx.x = xprior;
                }
            }
        } else {
            // generate prior from 3d neighbor
        }

        vkps.push_back(kp.px);
        vpriors.push_back(priorpx);
        vkpids.push_back(kp.mpid);
    }

    // step2: stereo tracking
    std::vector<cv::Point2f> vgoodlkps;
    std::vector<cv::Point2f> vgoodrkps;
    std::vector<uint64_t> vgoodids;
    vgoodlkps.reserve(nbkps);
    vgoodrkps.reserve(nbkps);
    vgoodids.reserve(nbkps);

    // step2.1: stereo tracking based on 3d prior
    if (!v3dpriors.empty()) {
        size_t nbpyrlvl = _config.stereo_match_3d_klt_pyr_lvl;  // only two level
        if (kf->pyr_left.size() < 2 * (nbpyrlvl + 1)) {
            nbpyrlvl = kf->pyr_left.size() / 2 - 1;
        }
        std::vector<bool> vkpstatus;
        _tracker->forwardBackwardKltTracking(
                kf->pyr_left, kf->pyr_right, nbpyrlvl, v3dkps, v3dpriors, vkpstatus);
        size_t nbgood = 0;
        size_t nb3dkps = v3dkps.size();
        for (size_t i = 0; i < nb3dkps; i++) {
            if (vkpstatus.at(i)) {
                vgoodlkps.push_back(v3dkps.at(i));
                vgoodrkps.push_back(v3dpriors.at(i));
                vgoodids.push_back(v3dkpids.at(i));
                nbgood++;
            } else {
                vkpids.push_back(v3dkpids.at(i));
                vkps.push_back(v3dkps.at(i));
                vpriors.push_back(v3dpriors.at(i));
            }
        }

        LOG(ERROR) << "stereo matching (KLT tracking) of KF " << kf->id
                   << " on 3d priors: " << nbgood << " out of " << nb3dkps << " kps tracked!\n";
    }

    // step2.2: stereo tracking based on 2d prior
    if (!vkps.empty()) {
        std::vector<bool> vkpstatus;
        _tracker->forwardBackwardKltTracking(
                kf->pyr_left, kf->pyr_right, _config.klt_pyr_lvl, vkps, vpriors, vkpstatus);
        size_t nbgood = 0;
        size_t nbkps = vkps.size();
        for (size_t i = 0; i < nbkps; i++) {
            if (vkpstatus[i]) {
                vgoodlkps.push_back(vkps.at(i));
                vgoodrkps.push_back(vpriors.at(i));
                vgoodids.push_back(vkpids.at(i));
                nbgood++;
            }
        }

        LOG(ERROR) << "stereo matching (KLT tracking) of KF " << kf->id
                   << " on 2d priors: " << nbgood << " out of " << nbkps << " kps tracked!\n";
    }

    // step3: check based on epipolar constraints
    std::unordered_set<uint64_t> sgoodids;
    for (size_t i = 0; i < vgoodids.size(); i++) {
        float epi_err = 0;
        auto& lpx = vgoodlkps.at(i);
        auto& rpx = vgoodrkps.at(i);
        if (_config.stereo_rect) {
            epi_err = fabs(lpx.y - rpx.y);
            // don't forget modify vgoodrkps!!!
            rpx.y = lpx.y;
        } else {
            auto lunpx = kf->camLeft()->undistortPx(lpx);
            auto runpx = kf->camRight()->undistortPx(rpx);
            epi_err = MultiViewGeometry::computeSampsonDistance(kf->Frl(), lunpx, runpx);
        }
        if (epi_err <= _config.stereo_match_epi_err) {
            kf->updateKeypointStereo(vgoodids.at(i), vgoodrkps.at(i));
            sgoodids.insert(vgoodids.at(i));
        }
    }

    LOG(ERROR) << "stereo matching of KF " << kf->id << " : get " << sgoodids.size()
               << " new stereo kps";

    if (_config.debugger.debug && !_config.debugger.stereo_matching.empty()) {
        std::string im_debug_path =
                _config.debugger.stereo_matching + "/" + std::to_string(kf->time) + ".png";
        cv::Mat im_debug_left = kf->im_left.clone();
        cv::Mat im_debug_right = kf->im_right.clone();
        cv::cvtColor(im_debug_left, im_debug_left, cv::COLOR_GRAY2BGR);
        cv::cvtColor(im_debug_right, im_debug_right, cv::COLOR_GRAY2BGR);

        for (size_t i = 0; i < v3dkps.size(); i++) {
            if (sgoodids.count(v3dkpids.at(i))) {
                cv::circle(im_debug_left, v3dkps[i], 3, cv::Scalar(255, 0, 0), -1);
                cv::circle(im_debug_right, v3dpriors[i], 3, cv::Scalar(255, 0, 0), -1);
                cv::line(im_debug_right, v3dkps[i], v3dpriors[i], cv::Scalar(255, 0, 0));
            }
        }
        for (size_t i = 0; i < vkps.size(); i++) {
            if (sgoodids.count(vkpids.at(i))) {
                cv::circle(im_debug_left, vkps[i], 3, cv::Scalar(0, 255, 0), -1);
                cv::circle(im_debug_right, vpriors[i], 3, cv::Scalar(0, 255, 0), -1);
                cv::line(im_debug_right, vkps[i], vpriors[i], cv::Scalar(0, 255, 0));
            } else {
                cv::circle(im_debug_left, vkps[i], 3, cv::Scalar(0, 0, 255), -1);
                cv::circle(im_debug_right, vpriors[i], 3, cv::Scalar(0, 0, 255), -1);
                cv::line(im_debug_right, vkps[i], vpriors[i], cv::Scalar(0, 0, 255));
            }
        }
        cv::hconcat(im_debug_left, im_debug_right, im_debug_right);
        cv::putText(im_debug_right,
                    "blue: 3Dmatch",
                    cv::Point2f(10, 30),
                    cv::FONT_HERSHEY_PLAIN,
                    1.4,
                    cv::Scalar(255, 0, 255),
                    2);
        cv::putText(im_debug_right,
                    "green: 2Dmatch",
                    cv::Point2f(10, 55),
                    cv::FONT_HERSHEY_PLAIN,
                    1.4,
                    cv::Scalar(255, 0, 255),
                    2);
        cv::putText(im_debug_right,
                    "red: no match",
                    cv::Point2f(10, 80),
                    cv::FONT_HERSHEY_PLAIN,
                    1.4,
                    cv::Scalar(255, 0, 255),
                    2);
        cv::imwrite(im_debug_path, im_debug_right);
    }
}

void Backend::triangulateStereo(const Frame::Ptr kf) {
    std::vector<Keypoint> vkps;
    kf->getKeypointsStereo(vkps);
    Sophus::SE3d Trl = kf->camRight()->Tcic0();
    size_t nbkps = vkps.size();
    size_t nbstereos = 0;
    size_t nbgood = 0;
    for (size_t i = 0; i < nbkps; i++) {
        auto& kp = vkps[i];
        if (!kp.is_3d && kp.is_stereo) {
            nbstereos++;
            Eigen::Vector3d left_pt = MultiViewGeometry::triangulate(Trl, kp.bv, kp.rbv);
            Eigen::Vector3d right_pt = Trl * left_pt;

            auto lpx_rpj = kf->projWorldToUnpx(left_pt);
            auto rpx_rpj = kf->projWorldToRightUnpx(left_pt);
            auto ldist = cv::norm(lpx_rpj - kp.unpx);
            auto rdist = cv::norm(rpx_rpj - kp.runpx);
            bool rpj_outlier = ldist > _config.max_reproj_dist || rdist > _config.max_reproj_dist;

            auto mp = _map->getMappoint(kp.mpid);
            if (mp) {
                if (left_pt.z() > 0.1 && right_pt.z() > 0.1 && !rpj_outlier) {
                    mp->setPos(kf->Twc() * left_pt);
                    for (auto& kf_it : mp->getKfObs()) {
                        kf_it.second->turnKeypoint3d(kp.mpid);
                    }
                    nbgood++;
                } else {
                    kf->removeKeypointStereo(kp.mpid);
                }
            } else {
                kf->removeKeypoint(kp.mpid);
            }
        }
    }
    LOG(ERROR) << "KF " << kf->id << ": triangulate " << nbgood << " points from " << nbstereos
               << " stereo points";
}

}  // namespace vo_lab
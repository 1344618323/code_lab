#include "frontend.h"
#include <glog/logging.h>
#include <unordered_set>
#include "multi_view_geometry.h"

namespace vo_lab {

void MotionModel::predict(Sophus::SE3d& Twc, double time) {
    if (_prev_time < 0) {
        Twc = _prev_Twc;
        return;
    }
    auto dt = time - _prev_time;
    Twc = _prev_Twc * Sophus::SE3d::exp(_log_relT * dt);
}

void MotionModel::update(const Sophus::SE3d& Twc, double time) {
    if (_prev_time > 0) {
        _log_relT = (_prev_Twc.inverse() * Twc).log();
    } else {
        _log_relT.setZero();
    }
    _prev_Twc = Twc;
    _prev_time = time;
}

Frontend::Frontend(const Config& config,
                   const std::shared_ptr<Map> map,
                   const cv::Ptr<cv::CLAHE> clahe,
                   const FeatureExtractor::Ptr extractor,
                   const FeatureTracker::Ptr tracker)
    : _config(config), _map(map), _clahe(clahe), _extractor(extractor), _tracker(tracker) {}

bool Frontend::tracking(const std::shared_ptr<Frame> curframe) {
    preprocessFrame(curframe);
    bool is_kf = true;
    if (_prevframe) {
        kltTracking(curframe);
        epipolar2d2dFiltering(curframe);
    }
    if (is_kf) {
        inheritOldKeypoint(curframe);
        createNewKeypoint(curframe);
        _map->addKeyframe(curframe);
        _prev_kf = curframe;
    }
    _prevframe = curframe;
    return true;
}

void Frontend::preprocessFrame(const Frame::Ptr frame) {
    _clahe->apply(frame->im_left_raw, frame->im_left);
    // be careful with `cv::buildOpticalFlowPyramid`:
    // `maxLevel`: 0-based maximal pyramid level number
    // `withDerivatives`: default value is true. precompute gradients for the every pyramid level.
    // If pyramid is constructed without the gradients then calcOpticalFlowPyrLK will calculate them
    // internally. This means that when using calcOpticalFlowPyrLK, you should set `withDerivatives`
    // to true.
    // Output Vector 0, 2, 4, ... representing images with decreasing resolution.
    // If `maxLevel` is 1, then the size of output vector is 4.
    cv::buildOpticalFlowPyramid(frame->im_left,
                                frame->pyr_left,
                                cv::Size(_config.klt_winsize, _config.klt_winsize),
                                _config.klt_pyr_lvl);

    Sophus::SE3d Twc;
    _motion_model.predict(Twc, frame->time);
    frame->setTwc(Twc);
    LOG(ERROR) << "Motion model predict the new pose of frame " << frame->id
               << " is t: " << Twc.translation().transpose()
               << "; q: " << Twc.unit_quaternion().coeffs().transpose();
}

void Frontend::kltTracking(const Frame::Ptr frame) {
    if (!_prevframe) {
        return;
    }
    std::vector<Keypoint> vkps;
    std::vector<size_t> v3dkpidx;
    std::vector<cv::Point2f> v3dkps;
    std::vector<cv::Point2f> v3dpriors;
    std::vector<size_t> v2dkpidx;
    std::vector<cv::Point2f> v2dkps;
    std::vector<cv::Point2f> v2dpriors;
    _prevframe->getKeypoints(vkps);
    v3dkpidx.reserve(vkps.size());
    v3dkps.reserve(vkps.size());
    v3dpriors.reserve(vkps.size());
    v2dkpidx.reserve(vkps.size());
    v2dkps.reserve(vkps.size());
    v2dpriors.reserve(vkps.size());
    for (size_t i = 0; i < vkps.size(); i++) {
        const auto& kp = vkps[i];
        if (_config.klt_use_prior && kp.is_3d) {
            auto mp = _map->getMappoint(kp.mpid);
            Eigen::Vector3d mp_pos;
            if (mp->getPos(mp_pos)) {
                auto prior_px = frame->projWorldToPx(mp_pos);
                if (frame->isInImage(prior_px)) {
                    v3dkps.push_back(kp.px);
                    v3dpriors.push_back(prior_px);
                    v3dkpidx.push_back(i);
                    continue;
                }
            }
        }
        v2dkps.push_back(kp.px);
        v2dkpidx.push_back(i);
    }

    std::vector<bool> v3dstatus;
    if (!v3dkps.empty()) {
        size_t nb3dgood = 0;
        _tracker->forwardBackwardKltTracking(_prevframe->pyr_left,
                                             frame->pyr_left,
                                             _config.klt_pyr_lvl_prior,
                                             v3dkps,
                                             v3dpriors,
                                             v3dstatus);

        for (size_t i = 0; i < v3dkps.size(); i++) {
            if (v3dstatus[i]) {
                frame->addKeypoint(
                        vkps[v3dkpidx[i]].mpid, v3dpriors[i], cv::Mat(), vkps[v3dkpidx[i]].is_3d);
                nb3dgood++;
            } else {
                v2dkps.push_back(v3dkps[i]);
                v2dkpidx.push_back(v3dkpidx[i]);
            }
        }

        LOG(ERROR) << "KLT traking from frame " << _prevframe->id << " to frame " << frame->id
                   << " on 3d priors: " << nb3dgood << " out of " << v3dstatus.size()
                   << " kps tracked!\n";
    }

    v2dpriors = v2dkps;
    std::vector<bool> v2dstatus;
    size_t nb2dgood = 0;
    _tracker->forwardBackwardKltTracking(_prevframe->pyr_left,
                                         frame->pyr_left,
                                         _config.klt_pyr_lvl,
                                         v2dkps,
                                         v2dpriors,
                                         v2dstatus);
    for (size_t i = 0; i < v2dkps.size(); i++) {
        if (v2dstatus[i]) {
            frame->addKeypoint(
                    vkps[v2dkpidx[i]].mpid, v2dpriors[i], cv::Mat(), vkps[v2dkpidx[i]].is_3d);
            nb2dgood++;
        }
    }
    LOG(ERROR) << "KLT traking from frame " << _prevframe->id << " to frame " << frame->id
               << " on 2d priors: " << nb2dgood << " out of " << v2dstatus.size()
               << " kps tracked!\n";

    if (_config.debugger.debug && !_config.debugger.klt_tracking.empty()) {
        std::string im_debug_path =
                _config.debugger.klt_tracking + "/" + std::to_string(frame->time) + ".png";
        cv::Mat prev_im_debug = _prevframe->im_left.clone();
        cv::Mat cur_im_debug = frame->im_left.clone();
        cv::cvtColor(prev_im_debug, prev_im_debug, cv::COLOR_GRAY2BGR);
        cv::cvtColor(cur_im_debug, cur_im_debug, cv::COLOR_GRAY2BGR);
        for (size_t i = 0; i < v3dkps.size(); i++) {
            if (v3dstatus[i]) {
                cv::circle(prev_im_debug, v3dkps[i], 3, cv::Scalar(0, 255, 0), -1);
                cv::circle(cur_im_debug, v3dpriors[i], 3, cv::Scalar(0, 255, 0), -1);
                cv::line(cur_im_debug, v3dkps[i], v3dpriors[i], cv::Scalar(0, 255, 0));
            }
        }
        for (size_t i = 0; i < v2dkps.size(); i++) {
            if (v2dstatus[i]) {
                cv::circle(prev_im_debug, v2dkps[i], 3, cv::Scalar(0, 0, 255), -1);
                cv::circle(cur_im_debug, v2dpriors[i], 3, cv::Scalar(0, 0, 255), -1);
                cv::line(cur_im_debug, v2dkps[i], v2dpriors[i], cv::Scalar(0, 0, 255));
            }
        }
        cv::hconcat(prev_im_debug, cur_im_debug, cur_im_debug);
        cv::imwrite(im_debug_path, cur_im_debug);
    }
}

void Frontend::epipolar2d2dFiltering(const Frame::Ptr frame) {
    if (!_prev_kf) {
        return;
    }
    std::vector<Keypoint> curkps;
    frame->getKeypoints(curkps);
    std::vector<Keypoint> prevkps;
    std::vector<Eigen::Vector3d> curbv, prevbv, cur3dbv, prev3dbv;

    // xxxidx[i] = j. i corresponds to idx of `xxxbv`, j corresponds to idx of `xxxkps`
    std::vector<size_t> curbv_idx, prevbv_idx, cur3dbv_idx, prev3dbv_idx;

    curbv.reserve(curkps.size());
    prevbv.reserve(curkps.size());
    cur3dbv.reserve(curkps.size());
    prev3dbv.reserve(curkps.size());
    curbv_idx.reserve(curkps.size());
    prevbv_idx.reserve(curkps.size());
    cur3dbv_idx.reserve(curkps.size());
    prev3dbv_idx.reserve(curkps.size());
    size_t j = 0;
    for (size_t i = 0; i < curkps.size(); i++) {
        const auto& kp = curkps[i];
        Keypoint prevkp;
        if (!_prev_kf->getKeypoint(kp.mpid, prevkp)) {
            continue;
        }
        prevkps.push_back(prevkp);
        curbv.push_back(kp.bv);
        prevbv.push_back(prevkp.bv);
        curbv_idx.push_back(i);
        prevbv_idx.push_back(j);
        if (kp.is_3d) {
            cur3dbv.push_back(kp.bv);
            prev3dbv.push_back(prevkp.bv);
            cur3dbv_idx.push_back(i);
            prev3dbv_idx.push_back(j);
        }
        j++;
    }

    bool epi_from_3dkps = false;
    if (_config.epi_from_3d && _config.stereo && cur3dbv.size() > 30) {
        epi_from_3dkps = true;
    }

    // solving essential matrix requires at least 5 points
    if (curbv.size() < 8) {
        LOG(ERROR) << "Skip epi filter due to insufficient quantity of matched kps "
                   << curbv.size();
        return;
    }

    auto& ref_cbv = epi_from_3dkps ? cur3dbv : curbv;
    auto& ref_pbv = epi_from_3dkps ? prev3dbv : prevbv;
    auto& ref_cbv_idx = epi_from_3dkps ? cur3dbv_idx : curbv_idx;
    auto& ref_pbv_idx = epi_from_3dkps ? prev3dbv_idx : prevbv_idx;
    Sophus::SO3d Rotpc = (_prev_kf->Tcw() * frame->Twc()).so3();
    double avg_parallax = 0;
    for (size_t i = 0; i < ref_cbv.size(); i++) {
        cv::Point2f rotpx = frame->camLeft()->projCamToPx(Rotpc * ref_cbv[i]);
        avg_parallax += cv::norm(rotpx - prevkps[ref_pbv_idx[i]].unpx);
    }
    avg_parallax /= ref_cbv.size();

    // the parallax is too small, which is bad for solving essential matrix
    if (avg_parallax < 2 * _config.epi_errth) {
        LOG(ERROR) << "Skip epi filter due to small parallax " << avg_parallax;
        return;
    }

    bool use_epi_motion = false;
    if (!_config.stereo && cur3dbv.size() < 30) {
        use_epi_motion = true;
    }

    float f = (frame->camLeft()->K()(0, 0) + frame->camLeft()->K()(1, 1)) / 2;
    Eigen::Matrix3d Rpc;
    Eigen::Vector3d tpc;

    // v[i] = j, use outlier_bv_idx[i] to find ref_cbv[j]/ref_pbv[j]/ref_cbv_idx[j]/ref_pbv_idx[j]
    std::vector<size_t> outlier_bv_idx;

    bool epi_success = MultiViewGeometry::essentialMatrix(
            ref_cbv, ref_pbv, Rpc, tpc, f, _config.epi_errth, use_epi_motion, outlier_bv_idx);

    LOG(ERROR) << "Calc Epi matrix from " << ref_cbv.size() << (epi_from_3dkps ? " 3d" : " ")
               << "pts. outlier nb: " << outlier_bv_idx.size();
    if (!epi_success || outlier_bv_idx.size() > 0.5 * ref_cbv.size()) {
        LOG(ERROR) << "Too many outliers, skip epi filter as meight be a degenerate case";
        return;
    }

    if (use_epi_motion && _map->kfs().size() > 2) {
        Sophus::SE3d Tpw = _prev_kf->Tcw();
        Sophus::SE3d Tpc = Tpw * frame->Twc();
        double scale = Tpc.translation().norm();
        tpc.normalize();
        Tpc = Sophus::SE3d(Rpc, tpc * scale);
        frame->setTwc(_prev_kf->Twc() * Tpc);
    }

    // remove outliers
    std::unordered_set<uint64_t> outlier_mpids;
    for (auto& idx : outlier_bv_idx) {
        auto& bad_kp = curkps[ref_cbv_idx[idx]];
        frame->removeKeypoint(bad_kp.mpid);
        outlier_mpids.insert(bad_kp.mpid);
    }

    if (epi_from_3dkps) {
        Eigen::Matrix3d Fpc = MultiViewGeometry::fundamentalMatrix(
                _prev_kf->Twc(), frame->Twc(), _prev_kf->camLeft()->K(), frame->camLeft()->K());
        for (size_t i = 0; i < curbv.size(); i++) {
            const auto& curkp = curkps[curbv_idx[i]];
            if (curkp.is_3d) {
                continue;  // has been processed
            }
            const auto& prevkp = prevkps[prevbv_idx[i]];
            float epi_err = MultiViewGeometry::computeSampsonDistance(Fpc, curkp.unpx, prevkp.unpx);
            if (epi_err > _config.epi_errth) {
                frame->removeKeypoint(curkp.mpid);
                outlier_mpids.insert(curkp.mpid);
            }
        }
    }
    LOG(ERROR) << "Epi filter " << outlier_mpids.size() << " outliers from " << curbv.size()
               << " kps";

    if (_config.debugger.debug && !_config.debugger.epi_filter.empty()) {
        std::string im_debug_path =
                _config.debugger.epi_filter + "/" + std::to_string(frame->time) + ".png";
        cv::Mat prev_im_debug = _prev_kf->im_left.clone();
        cv::Mat cur_im_debug = frame->im_left.clone();
        cv::cvtColor(prev_im_debug, prev_im_debug, cv::COLOR_GRAY2BGR);
        cv::cvtColor(cur_im_debug, cur_im_debug, cv::COLOR_GRAY2BGR);
        for (size_t i = 0; i < curbv.size(); i++) {
            const auto& curkp = curkps[curbv_idx[i]];
            const auto& prevkp = prevkps[prevbv_idx[i]];
            cv::Scalar color(0, 0, 0);
            if (!outlier_mpids.count(curkp.mpid)) {
                color(0) = 255;
            } else if (curkp.is_3d) {
                color(1) = 255;
            } else {
                color(2) = 255;
            }
            cv::circle(prev_im_debug, prevkp.px, 3, color, -1);
            cv::circle(cur_im_debug, curkp.px, 3, color, -1);
            cv::line(cur_im_debug, prevkp.px, curkp.px, color);
        }
        cv::hconcat(prev_im_debug, cur_im_debug, cur_im_debug);
        cv::imwrite(im_debug_path, cur_im_debug);
    }
}

void Frontend::inheritOldKeypoint(const Frame::Ptr kf) {
    std::vector<std::set<uint64_t>> kpid_grid;
    size_t nbkps;
    kf->getKeppointIdsGrid(kpid_grid, nbkps);
    if (nbkps > kpid_grid.size()) {
        for (auto& kpids : kpid_grid) {
            if (kpids.size() > 2) {
                std::vector<std::pair<size_t, uint64_t>> obs_kpid;
                for (auto& kpid : kpids) {
                    auto mp = _map->getMappoint(kpid);
                    if (mp) {
                        size_t obs = mp->getKfObs().size();
                        obs_kpid.emplace_back(obs, kpid);
                    } else {
                        obs_kpid.emplace_back(0, kpid);
                    }
                }
                std::sort(obs_kpid.begin(),
                          obs_kpid.end(),
                          [](const std::pair<size_t, uint64_t>& l,
                             const std::pair<size_t, uint64_t>& r) { return l.first < r.first; });
                int del_n = obs_kpid.size() - 2;
                for (auto& p : obs_kpid) {
                    if (p.first == 0 || del_n > 0) {
                        kf->removeKeypoint(p.second);
                        del_n--;
                    }
                }
            }
        }
    }

    // associate with mappoint
    std::vector<Keypoint> kps;
    kf->getKeypoints(kps);
    for (auto& kp : kps) {
        auto mp = _map->getMappoint(kp.mpid);
        if (mp) {
            mp->addKfObs(kf, kp.desc);
        } else {
            kf->removeKeypoint(kp.mpid);
        }
    }
}

bool Frontend::createNewKeypoint(const Frame::Ptr kf) {
    std::string im_debug_path;
    if (_config.debugger.debug && !_config.debugger.kps_extract.empty()) {
        im_debug_path = _config.debugger.kps_extract + "/" + std::to_string(kf->time) + ".png";
    }
    std::vector<Keypoint> voldkps;
    kf->getKeypoints(voldkps);
    std::vector<cv::Point2f> voldpts(voldkps.size());
    for (size_t i = 0; i < voldkps.size(); i++) {
        voldpts[i] = voldkps[i].px;
    }
    std::vector<cv::Point2f> vnewkps;
    size_t nbgood = 0;
    _extractor->detect(kf->im_left, voldpts, vnewkps, kf->camLeft()->roi_mask(), im_debug_path);
    if (!vnewkps.empty()) {
        std::vector<cv::Mat> descs;
        _extractor->describeBRIEF(kf->im_left_raw, vnewkps, descs);
        for (size_t i = 0; i < vnewkps.size(); i++) {
            if (kf->addKeypoint(_mpid, vnewkps[i], descs.at(i))) {
                Mappoint::Ptr mp = std::make_shared<Mappoint>(_mpid, kf, descs.at(i));
                _mpid++;
                _map->addMappoint(mp);
                nbgood++;
            }
        }
        LOG(ERROR) << "KF " << kf->id << " already has " << voldpts.size() << " old kps, and add "
                   << nbgood << " new kps";
    }

    return true;
}

}  // namespace vo_lab

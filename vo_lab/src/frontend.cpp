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
        computePose(curframe);
    }
    if (is_kf) {
        inheritOldKeypoint(curframe);
        createNewKeypoint(curframe);
        _map->addKeyframe(curframe);
        _prev_kf = curframe;
    }
    _prevframe = curframe;
    _motion_model.update(curframe->Twc(), curframe->time);
    return true;
}

void Frontend::preprocessFrame(const Frame::Ptr curframe) {
    _clahe->apply(curframe->im_left_raw, curframe->im_left);
    // be careful with `cv::buildOpticalFlowPyramid`:
    // `maxLevel`: 0-based maximal pyramid level number
    // `withDerivatives`: default value is true. precompute gradients for the every pyramid level.
    // If pyramid is constructed without the gradients then calcOpticalFlowPyrLK will calculate them
    // internally. This means that when using calcOpticalFlowPyrLK, you should set `withDerivatives`
    // to true.
    // Output Vector 0, 2, 4, ... representing images with decreasing resolution.
    // If `maxLevel` is 1, then the size of output vector is 4.
    cv::buildOpticalFlowPyramid(curframe->im_left,
                                curframe->pyr_left,
                                cv::Size(_config.klt_winsize, _config.klt_winsize),
                                _config.klt_pyr_lvl);

    Sophus::SE3d Twc;
    _motion_model.predict(Twc, curframe->time);
    curframe->setTwc(Twc);
    LOG(ERROR) << "Motion model predict the pose of curframe " << curframe->id
               << " is t: " << Twc.translation().transpose()
               << "; q: " << Twc.unit_quaternion().coeffs().transpose();
}

void Frontend::kltTracking(const Frame::Ptr curframe) {
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
        auto mp = kp.mp.lock();
        if (!mp) {
            continue;
        }
        Eigen::Vector3d mp_pos;
        if (_config.klt_use_prior && mp->getPos(mp_pos)) {
            auto prior_px = curframe->projWorldToPx(mp_pos);
            if (curframe->isInImage(prior_px)) {
                v3dkps.push_back(kp.px);
                v3dpriors.push_back(prior_px);
                v3dkpidx.push_back(i);
                continue;
            }
        }
        v2dkps.push_back(kp.px);
        v2dkpidx.push_back(i);
    }

    std::vector<bool> v3dstatus;
    if (!v3dkps.empty()) {
        size_t nb3dgood = 0;
        _tracker->forwardBackwardKltTracking(_prevframe->pyr_left,
                                             curframe->pyr_left,
                                             _config.klt_pyr_lvl_prior,
                                             v3dkps,
                                             v3dpriors,
                                             v3dstatus);

        for (size_t i = 0; i < v3dkps.size(); i++) {
            if (v3dstatus[i]) {
                if (curframe->addKeypoint(vkps[v3dkpidx[i]].mp.lock(), v3dpriors[i], cv::Mat())) {
                    nb3dgood++;
                }
            } else {
                v2dkps.push_back(v3dkps[i]);
                v2dkpidx.push_back(v3dkpidx[i]);
            }
        }

        LOG(ERROR) << "KLT traking from prevframe " << _prevframe->id << " to curframe "
                   << curframe->id << " on 3d priors: " << nb3dgood << " out of "
                   << v3dstatus.size() << " kps tracked!\n";
    }

    v2dpriors = v2dkps;
    std::vector<bool> v2dstatus;
    size_t nb2dgood = 0;
    _tracker->forwardBackwardKltTracking(_prevframe->pyr_left,
                                         curframe->pyr_left,
                                         _config.klt_pyr_lvl,
                                         v2dkps,
                                         v2dpriors,
                                         v2dstatus);
    for (size_t i = 0; i < v2dkps.size(); i++) {
        if (v2dstatus[i] &&
            curframe->addKeypoint(vkps[v2dkpidx[i]].mp.lock(), v2dpriors[i], cv::Mat())) {
            nb2dgood++;
        }
    }
    LOG(ERROR) << "KLT traking from prevframe " << _prevframe->id << " to curframe " << curframe->id
               << " on 2d priors: " << nb2dgood << " out of " << v2dstatus.size()
               << " kps tracked!\n";

    if (_config.debugger.debug && !_config.debugger.klt_tracking.empty()) {
        std::string im_debug_path =
                _config.debugger.klt_tracking + "/" + std::to_string(curframe->time) + ".png";
        cv::Mat prev_im_debug = _prevframe->im_left.clone();
        cv::Mat cur_im_debug = curframe->im_left.clone();
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
        cv::putText(cur_im_debug,
                    "red: 2Dmatch",
                    cv::Point2f(10, 30),
                    cv::FONT_HERSHEY_PLAIN,
                    1.4,
                    cv::Scalar(255, 0, 255),
                    2);
        cv::putText(cur_im_debug,
                    "green: 3Dmatch",
                    cv::Point2f(10, 55),
                    cv::FONT_HERSHEY_PLAIN,
                    1.4,
                    cv::Scalar(255, 0, 255),
                    2);
        cv::imwrite(im_debug_path, cur_im_debug);
    }
}

void Frontend::epipolar2d2dFiltering(const Frame::Ptr curframe) {
    if (!_prev_kf) {
        return;
    }
    std::vector<Keypoint> curkps;
    curframe->getKeypoints(curkps);
    std::vector<Keypoint> prevkps;
    std::vector<Eigen::Vector3d> curbv, prevbv, cur3dbv, prev3dbv;

    // xxxidx[i] = j. i corresponds to idx of `xxxbv`, j corresponds to idx of `xxxkps`
    std::vector<size_t> curbv_idx, prevbv_idx, cur3dbv_idx, prev3dbv_idx;

    prevkps.reserve(curkps.size());
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
        if (kp.is3D()) {
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
    Sophus::SO3d Rotpc = (_prev_kf->Tcw() * curframe->Twc()).so3();
    double avg_parallax = 0;
    for (size_t i = 0; i < ref_cbv.size(); i++) {
        cv::Point2f rotpx = curframe->camLeft()->projCamToPx(Rotpc * ref_cbv[i]);
        avg_parallax += cv::norm(rotpx - prevkps[ref_pbv_idx[i]].unpx);
    }
    avg_parallax /= ref_cbv.size();

    // the parallax is too small, which is bad for solving essential matrix
    if (avg_parallax < 2 * _config.epi_errth) {
        LOG(ERROR) << "Skip epi filter due to small parallax (epi 3d " << epi_from_3dkps
                   << "): " << avg_parallax;
        return;
    }

    bool use_epi_motion = false;
    if (!_config.stereo && cur3dbv.size() < 30) {
        use_epi_motion = true;
    }

    float f = (curframe->camLeft()->K()(0, 0) + curframe->camLeft()->K()(1, 1)) / 2;
    Eigen::Matrix3d Rpc;
    Eigen::Vector3d tpc;

    // v[i] = j, use outlier_bv_idx[i] to find ref_cbv[j]/ref_pbv[j]/ref_cbv_idx[j]/ref_pbv_idx[j]
    std::vector<size_t> outlier_bv_idx;

    bool epi_success = MultiViewGeometry::essentialMatrix(
            ref_cbv, ref_pbv, Rpc, tpc, f, _config.epi_errth, use_epi_motion, outlier_bv_idx);

    LOG(ERROR) << "Epipolar filter: curframe " << curframe->id << " filter "
               << outlier_bv_idx.size() << " outliers from " << ref_cbv.size()
               << (epi_from_3dkps ? " 3d" : " ") << "kps";
    if (!epi_success || outlier_bv_idx.size() > 0.5 * ref_cbv.size()) {
        LOG(ERROR) << "Too many outliers, skip epi filter as meight be a degenerate case";
        return;
    }

    if (use_epi_motion && _map->kfs().size() > 2) {
        Sophus::SE3d Tpw = _prev_kf->Tcw();
        Sophus::SE3d Tpc = Tpw * curframe->Twc();
        double scale = Tpc.translation().norm();
        tpc.normalize();
        Tpc = Sophus::SE3d(Rpc, tpc * scale);
        curframe->setTwc(_prev_kf->Twc() * Tpc);
    }

    // remove outliers
    std::unordered_set<uint64_t> outlier_mpids;
    for (auto& idx : outlier_bv_idx) {
        auto& bad_kp = curkps[ref_cbv_idx[idx]];
        curframe->removeKeypoint(bad_kp.mpid);
        outlier_mpids.insert(bad_kp.mpid);
    }

    if (epi_from_3dkps) {
        Eigen::Matrix3d Fpc = MultiViewGeometry::fundamentalMatrix(_prev_kf->Twc(),
                                                                   curframe->Twc(),
                                                                   _prev_kf->camLeft()->K(),
                                                                   curframe->camLeft()->K());
        for (size_t i = 0; i < curbv.size(); i++) {
            const auto& curkp = curkps[curbv_idx[i]];
            if (curkp.is3D()) {
                continue;  // has been processed
            }
            const auto& prevkp = prevkps[prevbv_idx[i]];
            float epi_err = MultiViewGeometry::computeSampsonDistance(Fpc, curkp.unpx, prevkp.unpx);
            if (epi_err > _config.epi_errth) {
                curframe->removeKeypoint(curkp.mpid);
                outlier_mpids.insert(curkp.mpid);
            }
        }
        LOG(ERROR) << "Epipolar filter: curframe " << curframe->id << " filter "
                   << outlier_mpids.size() << " outliers from " << curbv.size() << " kps";
    }

    if (_config.debugger.debug && !_config.debugger.epi_filter.empty()) {
        std::string im_debug_path =
                _config.debugger.epi_filter + "/" + std::to_string(curframe->time) + ".png";
        cv::Mat prev_im_debug = _prev_kf->im_left.clone();
        cv::Mat cur_im_debug = curframe->im_left.clone();
        cv::cvtColor(prev_im_debug, prev_im_debug, cv::COLOR_GRAY2BGR);
        cv::cvtColor(cur_im_debug, cur_im_debug, cv::COLOR_GRAY2BGR);
        for (size_t i = 0; i < curbv.size(); i++) {
            const auto& curkp = curkps[curbv_idx[i]];
            const auto& prevkp = prevkps[prevbv_idx[i]];
            cv::Scalar color(0, 0, 0);
            if (outlier_mpids.count(curkp.mpid)) {
                color(0) = 255;  // outlier: blue
            } else if (curkp.is3D()) {
                color(1) = 255;  // 3d inlier: green
            } else {
                color(2) = 255;  // 2d inlier: red
            }
            cv::circle(prev_im_debug, prevkp.px, 3, color, -1);
            cv::circle(cur_im_debug, curkp.px, 3, color, -1);
            cv::line(cur_im_debug, prevkp.px, curkp.px, color);
        }
        cv::hconcat(prev_im_debug, cur_im_debug, cur_im_debug);
        cv::putText(cur_im_debug,
                    "red: 2Dinlier",
                    cv::Point2f(10, 30),
                    cv::FONT_HERSHEY_PLAIN,
                    1.4,
                    cv::Scalar(255, 0, 255),
                    2);
        cv::putText(cur_im_debug,
                    "green: 3Dinlier",
                    cv::Point2f(10, 55),
                    cv::FONT_HERSHEY_PLAIN,
                    1.4,
                    cv::Scalar(255, 0, 255),
                    2);
        cv::putText(cur_im_debug,
                    "blue: outlier",
                    cv::Point2f(10, 80),
                    cv::FONT_HERSHEY_PLAIN,
                    1.4,
                    cv::Scalar(255, 0, 255),
                    2);
        cv::imwrite(im_debug_path, cur_im_debug);
    }
}

void Frontend::computePose(const Frame::Ptr curframe) {
    std::vector<Keypoint> kps;
    curframe->getKeypoints(kps);
    std::vector<Eigen::Vector3d> wpts;
    std::vector<Eigen::Vector3d> bvs;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> pxs;
    wpts.reserve(kps.size());
    bvs.reserve(kps.size());
    pxs.reserve(pxs.size());
    for (auto& kp : kps) {
        auto mp = kp.mp.lock();
        Eigen::Vector3d wpt;
        if (mp && mp->getPos(wpt)) {
            wpts.push_back(wpt);
            bvs.push_back(kp.bv);
            pxs.emplace_back(kp.px.x, kp.px.y);
        }
    }
    if (wpts.empty()) {
        return;
    }

    Eigen::Matrix3d K = curframe->camLeft()->K();
    Sophus::SE3d Twc = curframe->Twc();
    std::vector<size_t> p3p_outlier_idx;
    if (_config.do_p3p) {
        bool p3p_success = MultiViewGeometry::P3PRansac(bvs,
                                                        wpts,
                                                        _config.p3p_ransac_iter,
                                                        _config.p3p_errth,
                                                        _config.p3p_optimize,
                                                        (K(0, 0) + K(1, 1)) / 2,
                                                        Twc,
                                                        p3p_outlier_idx);
        size_t nbinlier = bvs.size() - p3p_outlier_idx.size();
        if (!p3p_success || nbinlier < 4 || Twc.translation().array().isInf().any() ||
            Twc.translation().array().isNaN().any()) {
            LOG(ERROR) << "Fatal situation, clear all kps of curframe " << curframe->id;
            curframe->clearKeypoints();
            return;
        }
        curframe->setTwc(Twc);
        LOG(ERROR) << "P3P: curframe " << curframe->id << " filter " << p3p_outlier_idx.size()
                   << " outliers from " << bvs.size()
                   << " 3dkps. Twc t: " << Twc.translation().transpose()
                   << " ; q: " << Twc.unit_quaternion().coeffs().transpose();
    }

    decltype(wpts) wpts_pnp;
    decltype(pxs) pxs_pnp;
    wpts_pnp.reserve(wpts.size());
    pxs_pnp.reserve(wpts.size());
    size_t outlier_i = 0;
    size_t src_i = 0;
    for (; outlier_i < p3p_outlier_idx.size(); outlier_i++) {
        while (src_i < p3p_outlier_idx[outlier_i]) {
            wpts_pnp.push_back(wpts[src_i]);
            pxs_pnp.push_back(pxs[src_i]);
            src_i++;
        }
        src_i = p3p_outlier_idx[outlier_i] + 1;
    }
    while (src_i < wpts.size()) {
        wpts_pnp.push_back(wpts[src_i]);
        pxs_pnp.push_back(pxs[src_i]);
        src_i++;
    }

    std::vector<size_t> pnp_outlier_idx;

    if (_config.debugger.debug && !_config.debugger.pnp.empty()) {
        std::string im_debug_path =
                _config.debugger.pnp + "/" + std::to_string(curframe->time) + ".png";
        cv::Mat cur_im_debug = curframe->im_left.clone();
        cv::cvtColor(cur_im_debug, cur_im_debug, cv::COLOR_GRAY2BGR);
        for (size_t i = 0; i < p3p_outlier_idx.size(); i++) {
            auto px = pxs[p3p_outlier_idx[i]];
            cv::circle(cur_im_debug, cv::Point2f(px.x(), px.y()), 3, cv::Scalar(0, 0, 255), -1);
        }
        size_t inlier_i = 0;
        for (size_t outlier_i = 0; outlier_i < pnp_outlier_idx.size(); outlier_i++) {
            auto px = pxs_pnp[pnp_outlier_idx[outlier_i]];
            cv::circle(cur_im_debug, cv::Point2f(px.x(), px.y()), 3, cv::Scalar(0, 255, 0), -1);
            while (inlier_i < pnp_outlier_idx[outlier_i]) {
                auto px = pxs_pnp[inlier_i];
                cv::circle(cur_im_debug, cv::Point2f(px.x(), px.y()), 3, cv::Scalar(255, 0, 0), -1);
                inlier_i++;
            }
            inlier_i = pnp_outlier_idx[outlier_i] + 1;
        }
        while (inlier_i < pxs_pnp.size()) {
            auto px = pxs_pnp[inlier_i];
            cv::circle(cur_im_debug, cv::Point2f(px.x(), px.y()), 3, cv::Scalar(255, 0, 0), -1);
            inlier_i++;
        }
        cv::putText(cur_im_debug,
                    "red: p3p outlier",
                    cv::Point2f(10, 30),
                    cv::FONT_HERSHEY_PLAIN,
                    1.4,
                    cv::Scalar(255, 0, 255),
                    2);
        cv::putText(cur_im_debug,
                    "green: pnp outlier",
                    cv::Point2f(10, 55),
                    cv::FONT_HERSHEY_PLAIN,
                    1.4,
                    cv::Scalar(255, 0, 255),
                    2);
        cv::putText(cur_im_debug,
                    "blue: inlier",
                    cv::Point2f(10, 80),
                    cv::FONT_HERSHEY_PLAIN,
                    1.4,
                    cv::Scalar(255, 0, 255),
                    2);
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
                        kf->removeKeypoint(kpid);
                    }
                }
                size_t nb = obs_kpid.size();
                if (nb > 2) {
                    std::sort(
                            obs_kpid.begin(),
                            obs_kpid.end(),
                            [](const std::pair<size_t, uint64_t>& l,
                               const std::pair<size_t, uint64_t>& r) { return l.first < r.first; });
                    size_t i = 0;
                    while (nb > 2) {
                        kf->removeKeypoint(obs_kpid[i++].second);
                        nb--;
                    }
                }
            }
        }
    }

    // associate with mappoint
    std::vector<Keypoint> kps;
    kf->getKeypoints(kps);
    for (auto& kp : kps) {
        auto mp = kp.mp.lock();
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
            Mappoint::Ptr mp = std::make_shared<Mappoint>(_mpid, kf, descs.at(i));
            _mpid++;
            if (kf->addKeypoint(mp, vnewkps[i], descs.at(i))) {
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

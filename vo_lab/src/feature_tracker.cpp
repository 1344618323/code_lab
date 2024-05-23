#include "feature_tracker.h"

namespace vo_lab {
void FeatureTracker::forwardBackwardKltTracking(const std::vector<cv::Mat>& prevpyr,
                                                const std::vector<cv::Mat>& nextpyr,
                                                int pyrlvl,
                                                std::vector<cv::Point2f>& pts,
                                                std::vector<cv::Point2f>& prior_pts,
                                                std::vector<bool>& pt_status) const {
    if (pts.empty()) {
        return;
    }

    // ensure `pyrlvl` is valid
    if ((int)prevpyr.size() < 2 * (pyrlvl + 1)) {
        pyrlvl = prevpyr.size() / 2 - 1;
    }

    size_t nbpts = pts.size();
    pt_status.clear();
    pt_status.reserve(nbpts);

    std::vector<uchar> v_fwd_status;
    std::vector<float> v_fwd_err;
    std::vector<int> vptsidx;
    std::vector<cv::Point2f> v_fwd_pts;
    std::vector<cv::Point2f> v_bwd_pts;
    v_fwd_status.reserve(nbpts);
    v_fwd_err.reserve(nbpts);
    vptsidx.reserve(nbpts);
    v_fwd_pts.reserve(nbpts);
    v_bwd_pts.reserve(nbpts);

    // forward tracking
    cv::calcOpticalFlowPyrLK(prevpyr,
                             nextpyr,
                             pts,
                             prior_pts,
                             v_fwd_status,
                             v_fwd_err,
                             _klt_winsize,
                             pyrlvl,
                             _klt_convg_crit,
                             (cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS));

    size_t nbgood = 0;
    for (size_t i = 0; i < nbpts; i++) {
        if (!v_fwd_status.at(i)) {
            pt_status.push_back(false);
            continue;
        }

        // only useful when not set cv::OPTFLOW_LK_GET_MIN_EIGENVALS
        // if (v_fwd_err.at(i) > _klt_err) {
        //     pt_status.push_back(false);
        //     continue;
        // }
        if (cv::norm(pts.at(i) - prior_pts.at(i)) > _klt_err) {
            pt_status.push_back(false);
            continue;
        }

        if (!inBorder(prior_pts.at(i), nextpyr.at(0))) {
            pt_status.push_back(false);
            continue;
        }

        pt_status.push_back(true);
        v_fwd_pts.push_back(prior_pts.at(i));
        v_bwd_pts.push_back(pts.at(i));
        vptsidx.push_back(i);
        nbgood++;
    }
    if (v_fwd_pts.empty()) {
        return;
    }

    std::vector<uchar> v_bwd_status;
    std::vector<float> v_bwd_err;
    v_bwd_status.reserve(nbpts);
    v_bwd_err.reserve(nbpts);

    // backward
    cv::calcOpticalFlowPyrLK(nextpyr,
                             prevpyr,
                             v_fwd_pts,
                             v_bwd_pts,
                             v_bwd_status,
                             v_bwd_err,
                             _klt_winsize,
                             0,  // given the target, there is no need to use pyramid
                             _klt_convg_crit,
                             (cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS));
    nbgood = 0;
    for (int i = 0, iend = v_fwd_pts.size(); i < iend; i++) {
        int idx = vptsidx.at(i);
        if (!v_bwd_status.at(i)) {
            pt_status.at(idx) = false;
            continue;
        }
        if (cv::norm(pts.at(idx) - v_bwd_pts.at(i)) > _max_fbklt_dist) {
            pt_status.at(idx) = false;
            continue;
        }
        nbgood++;
    }
}

bool FeatureTracker::inBorder(const cv::Point2f& pt, const cv::Mat& im) const {
    return _border <= pt.x && pt.x < im.cols - _border && _border <= pt.y &&
           pt.y < im.rows - _border;
}

bool FeatureTracker::getLineMinSAD(const cv::Mat& iml,
                                   const cv::Mat& imr,
                                   const cv::Point2f& pt,
                                   const uint nwinsize,
                                   float& xprior,
                                   float& l1err,
                                   bool goleft) const {
    if (nwinsize <= 0 || nwinsize % 2 == 0) {
        return false;
    }
    int halfwin = nwinsize / 2;
    if (pt.x - halfwin < 0) {
        halfwin += (pt.x - halfwin);
    }
    if (pt.x + halfwin >= imr.cols) {
        halfwin -= (pt.x + halfwin - imr.cols - 1);
    }
    if (pt.y - halfwin < 0) {
        halfwin += (pt.y - halfwin);
    }
    if (pt.y + halfwin >= imr.rows) {
        halfwin -= (pt.y + halfwin - imr.rows - 1);
    }

    cv::Size winsize(2 * halfwin + 1, 2 * halfwin + 1);
    int nbwinpx = winsize.width * winsize.height;

    cv::Mat patch, target;
    cv::getRectSubPix(iml, winsize, pt, patch);

    float minsad = std::numeric_limits<float>::max();
    if (goleft) {
        for (float c = pt.x; c >= halfwin; c--) {
            cv::getRectSubPix(imr, winsize, cv::Point2f(c, pt.y), target);
            float err = cv::norm(patch, target, cv::NORM_L1);
            err /= nbwinpx;
            if (err < minsad) {
                minsad = err;
                xprior = c;
            }
        }
    } else {
        for (float c = pt.x; c < imr.cols - halfwin; c++) {
            cv::getRectSubPix(imr, winsize, cv::Point2f(c, pt.y), target);
            float err = cv::norm(patch, target, cv::NORM_L1);
            err /= nbwinpx;
            if (err < minsad) {
                minsad = err;
                xprior = c;
            }
        }
    }
    l1err = minsad;
    return true;
}
}  // namespace vo_lab
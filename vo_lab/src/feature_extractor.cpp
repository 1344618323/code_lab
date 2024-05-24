#include "feature_extractor.h"

namespace vo_lab {

void FeatureExtractor::detect(const cv::Mat& im,
                              const std::vector<cv::Point2f>& curkps,
                              std::vector<cv::Point2f>& newkps,
                              const cv::Mat& roi,
                              const std::string& im_debug_path) {
    newkps.clear();
    if (im.empty()) {
        return;
    }

    int radius = _cell_size / 4;
    int hcells = std::ceil((float)im.rows / _cell_size);
    int wcells = std::ceil((float)im.cols / _cell_size);
    int ncells = hcells * wcells;

    std::vector<bool> occ_cells(ncells, false);
    int nocc = 0;
    cv::Mat mask = roi.clone();
    if (mask.empty()) {
        mask = cv::Mat(im.rows, im.cols, CV_8U, cv::Scalar(255));
    }
    for (const auto& px : curkps) {
        int row = px.y / _cell_size;
        int col = px.x / _cell_size;
        int idx = row * wcells + col;
        if (!occ_cells[idx]) {
            occ_cells[idx] = true;
            cv::circle(mask, px, radius, cv::Scalar(0), -1);
            nocc++;
        }
    }

    // blur
    cv::Mat filtered_mat;
    cv::GaussianBlur(im, filtered_mat, cv::Size(3, 3), 0);

    // create a flattn grid that is divided into hcells*wcells
    std::vector<std::vector<cv::Point2f>> detected_kps(ncells);
    for (int quater_idx = 0; quater_idx < 4; quater_idx++) {
        int start_col = (quater_idx % 2);   // 0 1 0 1
        int start_row = (quater_idx >= 2);  // 0 0 1 1
        int half_wcells = std::ceil(wcells / 2.0);

// #define use_parallel
#ifdef use_parallel
        int half_hcells = std::ceil(hcells / 2.0);
        auto cvrange = cv::Range(0, half_wcells * half_hcells);
        parallel_for_(cvrange, [&](const cv::Range& range) {
            for (int i = range.start; i < range.end; i++) {
#else
        for (int i = 0; i < ncells; i++) {
#endif
                int col = start_col + (i % half_wcells) * 2;
                int row = start_row + (i / half_wcells) * 2;
                int cell_idx = row * wcells + col;
                int u = col * _cell_size;
                int v = row * _cell_size;

                if (u < im.cols && v < im.rows && !occ_cells[cell_idx]) {
                    cv::Rect cell_roi(u,
                                      v,
                                      std::min(_cell_size, im.cols - u),
                                      std::min(_cell_size, im.rows - v));
                    cv::Mat min_eigval_mat;
                    cv::cornerMinEigenVal(filtered_mat(cell_roi), min_eigval_mat, 3);

                    auto helper_fn = [&]() {
                        double dminval, dmaxval;
                        cv::Point minpt, maxpt;
                        cv::minMaxLoc(
                                min_eigval_mat, &dminval, &dmaxval, &minpt, &maxpt, mask(cell_roi));
                        if (dmaxval >= _max_quality) {
                            maxpt.x += u;
                            maxpt.y += v;
                            cv::circle(mask, maxpt, radius, cv::Scalar(0), -1);
                            detected_kps.at(cell_idx).push_back(maxpt);
                        }
                    };

                    helper_fn();
                    helper_fn();
                }
            }
#ifdef use_parallel
        });
#endif
    }

    for (const auto& v : detected_kps) {
        if (!v.empty()) {
            newkps.push_back(v.at(0));
        }
    }
    int nvisit = ncells - nocc;
    for (const auto& v : detected_kps) {
        if ((int)newkps.size() >= nvisit) {
            break;
        } else if (v.size() > 1) {
            newkps.push_back(v.at(1));
        }
    }

    // Update dmaxquality
    if (newkps.size() < 0.5 * nvisit) {
        _max_quality /= 2;
    } else if (newkps.size() > 0.9 * nvisit) {
        _max_quality *= 1.5;
    }

    // The accuracy is pixel level, compute Corners with Sub-Pixel Accuracy
    if (!newkps.empty()) {
        /// Set the need parameters to find the refined corners
        cv::Size winsize = cv::Size(3, 3);
        cv::Size zerozone = cv::Size(-1, -1);
        cv::TermCriteria criteria =
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01);
        cv::cornerSubPix(im, newkps, winsize, zerozone, criteria);
    }

    if (!im_debug_path.empty()) {
        cv::Mat im_debug;
        cv::cvtColor(im, im_debug, cv::COLOR_GRAY2BGR);
        for (auto& kp : curkps) {
            cv::circle(im_debug, kp, 3, cv::Scalar(0, 0, 255), -1);
        }
        for (auto& kp : newkps) {
            cv::circle(im_debug, kp, 3, cv::Scalar(0, 255, 0), -1);
        }
        for (int i = 1; i < hcells; i++) {
            cv::line(im_debug,
                     cv::Point(0, i * _cell_size),
                     cv::Point(im_debug.cols - 1, i * _cell_size),
                     cv::Scalar(255, 255, 255),
                     1);
        }
        for (int i = 1; i < wcells; i++) {
            cv::line(im_debug,
                     cv::Point(i * _cell_size, 0),
                     cv::Point(i * _cell_size, im_debug.rows - 1),
                     cv::Scalar(255, 255, 255),
                     1);
        }
        cv::putText(im_debug,
                    "red: oldkps",
                    cv::Point2f(10, 30),
                    cv::FONT_HERSHEY_PLAIN,
                    1.4,
                    cv::Scalar(255, 0, 255),
                    2);
        cv::putText(im_debug,
                    "green: newkps",
                    cv::Point2f(10, 55),
                    cv::FONT_HERSHEY_PLAIN,
                    1.4,
                    cv::Scalar(255, 0, 255),
                    2);
        cv::imwrite(im_debug_path, im_debug);
    }
}

void FeatureExtractor::describeBRIEF(const cv::Mat& im,
                                     const std::vector<cv::Point2f>& pts,
                                     std::vector<cv::Mat>& descs) {
    if (im.empty() || pts.empty()) {
        return;
    }
    descs.clear();
    descs.resize(pts.size());
    std::vector<cv::KeyPoint> kps;
    cv::KeyPoint::convert(pts, kps);
    cv::Mat kps_desc;
    _brief->compute(im, kps, kps_desc);

    // According to https://docs.opencv.org/3.4/d0/d13/classcv_1_1Feature2D.html, it is known that
    // the Feature2D->compute may change the kps, so an additional review is required to ensure that
    // the sizes of pts and descs are consistent

    size_t j = 0;
    for (size_t i = 0; i < pts.size(); i++) {
        if (j < kps.size()) {
            if (kps[j].pt == pts[i]) {
                descs.push_back(kps_desc.row(j));
                j++;
            } else {
                descs.push_back(cv::Mat());
            }
        } else {
            descs.push_back(cv::Mat());
        }
    }
}
}  // namespace vo_lab
#include "feature_extractor.h"

FeatureExtractor::FeatureExtractor(int nfast_th, double dmaxquality)
    : nfast_th_(nfast_th), dmaxquality_(dmaxquality) {
    pfast_ = cv::FastFeatureDetector::create(nfast_th_);
}

std::vector<cv::Point2f> FeatureExtractor::detectGridFAST(const cv::Mat& im,
                                                          const int ncellsize,
                                                          const std::vector<cv::Point2f>& vcurkps,
                                                          const cv::Mat& roi) {
    if (im.empty()) {
        return std::vector<cv::Point2f>();
    }

    size_t nhalfcell = ncellsize / 4;
    size_t nhcells = std::ceil((float)im.rows / ncellsize);
    size_t nwcells = std::ceil((float)im.cols / ncellsize);
    size_t nbcells = nhcells * nwcells;

    // mask vcurkps
    std::vector<std::vector<bool>> voccupcells(nhcells, std::vector<bool>(nwcells, false));
    cv::Mat mask = roi.clone();
    if (mask.empty()) {
        mask = cv::Mat(im.rows, im.cols, CV_8U, cv::Scalar(255));
    }
    for (const auto& px : vcurkps) {
        voccupcells[px.y / ncellsize][px.x / ncellsize] = true;
        cv::circle(mask, px, nhalfcell, cv::Scalar(0), -1);
    }

    // create a flattn grid that is divided into nhcells*nwcells
    std::vector<std::vector<cv::Point2f>> vdetectedp(nbcells);
    size_t nvisitd = 0;
    for (size_t quater_idx = 0; quater_idx < 4; quater_idx++) {
        size_t start_x = (quater_idx % 2);
        size_t start_y = (quater_idx >= 2);
        size_t half_wcells = std::ceil(nwcells / 2.0);
        size_t half_hcells = std::ceil(nhcells / 2.0);
        auto cvrange = cv::Range(0, half_wcells * half_hcells);
        parallel_for_(cvrange, [&](const cv::Range& range) {
            for (int i = range.start; i < range.end; i++) {
                int c = start_x + (i % half_wcells) * 2;
                int r = start_y + (i / half_wcells) * 2;
                int x = c * ncellsize;
                int y = r * ncellsize;
                if (x + ncellsize <= im.cols && y + ncellsize <= im.rows && !voccupcells[r][c]) {
                    nvisitd++;
                    cv::Rect hroi(x, y, ncellsize, ncellsize);
                    std::vector<cv::KeyPoint> vkps;
                    pfast_->detect(im(hroi), vkps, mask(hroi));
                    if (vkps.empty()) {
                        continue;
                    }
                    std::sort(vkps.begin(),
                              vkps.end(),
                              [](const cv::KeyPoint& p1, const cv::KeyPoint p2) {
                                  return p1.response > p2.response;
                              });
                    auto pt = vkps.at(0).pt;
                    pt.x += x;
                    pt.y += y;
                    cv::circle(mask, pt, nhalfcell, cv::Scalar(0), -1);
                    vdetectedp.at(r * nwcells + c).push_back(pt);
                }
            }
        });
    }

    std::vector<cv::Point2f> vnew_kps;
    for (const auto& v : vdetectedp) {
        if (!v.empty()) {
            vnew_kps.push_back(v.at(0));
        }
    }

    // Update FAST th.
    if (vnew_kps.size() < 0.5 * nvisitd && nvisitd > 10) {
        nfast_th_ *= 0.66;
        pfast_->setThreshold(nfast_th_);
    } else if (vnew_kps.size() == nvisitd) {
        nfast_th_ *= 1.5;
        pfast_->setThreshold(nfast_th_);
    }

    // The accuracy is pixel level, compute Corners with Sub-Pixel Accuracy
    if (!vnew_kps.empty()) {
        /// Set the need parameters to find the refined corners
        cv::Size winSize = cv::Size(3, 3);
        cv::Size zeroZone = cv::Size(-1, -1);
        cv::TermCriteria criteria =
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01);
        cv::cornerSubPix(im, vnew_kps, winSize, zeroZone, criteria);
    }

    return vnew_kps;
}

std::vector<cv::Point2f> FeatureExtractor::detectSingleScale(
        const cv::Mat& im,
        const int ncellsize,
        const std::vector<cv::Point2f>& vcurkps,
        const cv::Mat& roi) {
    if (im.empty()) {
        return std::vector<cv::Point2f>();
    }

    size_t nhalfcell = ncellsize / 4;
    size_t nhcells = std::ceil((float)im.rows / ncellsize);
    size_t nwcells = std::ceil((float)im.cols / ncellsize);
    size_t nbcells = nhcells * nwcells;

    // mask vcurkps
    std::vector<std::vector<bool>> voccupcells(nhcells, std::vector<bool>(nwcells, false));
    cv::Mat mask = roi.clone();
    if (mask.empty()) {
        mask = cv::Mat(im.rows, im.cols, CV_8U, cv::Scalar(255));
    }
    for (const auto& px : vcurkps) {
        voccupcells[px.y / ncellsize][px.x / ncellsize] = true;
        cv::circle(mask, px, nhalfcell, cv::Scalar(0), -1);
    }

    // blur
    cv::Mat filtered_mat;
    cv::GaussianBlur(im, filtered_mat, cv::Size(3, 3), 0);

    // create a flattn grid that is divided into nhcells*nwcells
    std::vector<std::vector<cv::Point2f>> vdetectedp(nbcells);
    size_t nvisitd = 0;
    for (size_t quater_idx = 0; quater_idx < 4; quater_idx++) {
        size_t start_x = (quater_idx % 2);
        size_t start_y = (quater_idx >= 2);
        size_t half_wcells = std::ceil(nwcells / 2.0);
        size_t half_hcells = std::ceil(nhcells / 2.0);
        auto cvrange = cv::Range(0, half_wcells * half_hcells);
        parallel_for_(cvrange, [&](const cv::Range& range) {
            for (int i = range.start; i < range.end; i++) {
                int c = start_x + (i % half_wcells) * 2;
                int r = start_y + (i / half_wcells) * 2;
                int x = c * ncellsize;
                int y = r * ncellsize;
                if (x + ncellsize <= im.cols && y + ncellsize <= im.rows && !voccupcells[r][c]) {
                    nvisitd++;
                    cv::Rect hroi(x, y, ncellsize, ncellsize);
                    cv::Mat hmap;
                    cv::cornerMinEigenVal(filtered_mat(hroi), hmap, 3);

                    auto helper_fn = [&]() {
                        double dminval, dmaxval;
                        cv::Point minpt, maxpt;
                        cv::minMaxLoc(hmap, &dminval, &dmaxval, &minpt, &maxpt, mask(hroi));
                        maxpt.x += x;
                        maxpt.y += y;
                        if (dmaxval >= dmaxquality_) {
                            cv::circle(mask, maxpt, nhalfcell, cv::Scalar(0), -1);
                            vdetectedp.at(r * nwcells + c).push_back(maxpt);
                        }
                    };

                    helper_fn();
                    helper_fn();
                }
            }
        });
    }

    std::vector<cv::Point2f> vnew_kps;
    for (const auto& v : vdetectedp) {
        if (!v.empty()) {
            vnew_kps.push_back(v.at(0));
        }
    }
    for (const auto& v : vdetectedp) {
        if (v.size() > 1) {
            vnew_kps.push_back(v.at(1));
            if (vnew_kps.size() == nvisitd) {
                break;
            }
        }
    }

    // Update dmaxquality
    if (vnew_kps.size() < 0.33 * nvisitd) {
        dmaxquality_ /= 2;
    } else if (vnew_kps.size() > 0.9 * nvisitd) {
        dmaxquality_ *= 1.5;
    }

    // The accuracy is pixel level, compute Corners with Sub-Pixel Accuracy
    if (!vnew_kps.empty()) {
        /// Set the need parameters to find the refined corners
        cv::Size winSize = cv::Size(3, 3);
        cv::Size zeroZone = cv::Size(-1, -1);
        cv::TermCriteria criteria =
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01);
        cv::cornerSubPix(im, vnew_kps, winSize, zeroZone, criteria);
    }

    return vnew_kps;
}

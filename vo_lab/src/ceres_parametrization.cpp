#include "ceres_parametrization.h"

namespace vo_lab {

bool ReprojectionErrorSE3::Evaluate(double const* const* parameters,
                                    double* residuals,
                                    double** jacobians) const {
    Eigen::Map<const Eigen::Vector3d> twc(parameters[0]);
    Eigen::Map<const Eigen::Quaterniond> qwc(parameters[0] + 3);
    Sophus::SE3d Twc(qwc, twc);
    Sophus::SE3d Tcw = Twc.inverse();
    Eigen::Vector3d cpt = Tcw * _wpt;
    
    return true;
}
}  // namespace vo_lab
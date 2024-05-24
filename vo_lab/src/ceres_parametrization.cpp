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
    double cpt_invz = 1 / cpt.z();
    Eigen::Vector2d pred(_fx * cpt.x() * cpt_invz + _cx, _fy * cpt.y() * cpt_invz + _cy);
    Eigen::Map<Eigen::Vector2d> err(residuals);
    err = _sqrt_info * (pred - _px);
    chi2err = err.squaredNorm();
    is_depth_positive = cpt_invz > 0;
    if (jacobians && jacobians[0]) {
        double cpt_invz2 = cpt_invz * cpt_invz;
        Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_px_cam;
        J_px_cam << cpt_invz * _fx, 0, -cpt_invz2 * _fx * cpt.x(), 0, cpt_invz * _fy,
                -cpt_invz2 * _fy * cpt.y();
        Eigen::Matrix<double, 2, 3, Eigen::RowMajor> B;
        // Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J;
    }
    return true;
}
}  // namespace vo_lab
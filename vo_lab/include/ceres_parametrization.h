#pragma once
#include <ceres/ceres.h>
#include <Eigen/Eigen>
#include <sophus/se3.hpp>

namespace vo_lab {

/*********** vertex *************/

/*
plus: x \oplus dx = exp(dx) x
dx \in R6
x \in R7
*/
class SE3LeftParameterization : public ceres::LocalParameterization {
  public:
    virtual int GlobalSize() const override { return 7; }
    virtual int LocalSize() const override { return 6; }
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
        Eigen::Map<const Eigen::Vector3d> t(x);
        Eigen::Map<const Eigen::Quaterniond> q(x + 3);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> dx(delta);  // delta = [rou, theta]
        Sophus::SE3d T = Sophus::SE3d::exp(dx) * Sophus::SE3d(q, t);
        Eigen::Map<Eigen::Vector3d> t_new(x_plus_delta);
        Eigen::Map<Eigen::Quaterniond> q_new(x_plus_delta + 3);
        t_new = T.translation();
        q_new = T.unit_quaternion();
        return true;
    };

    // d(x \oplus dx)/ d(dx)
    virtual bool ComputeJacobian(const double* x, double* jacobian) const override {
        // dGlobal/dlocal
        // Be careful! RowMajor! not ColMajor! (default value is ColMajor)
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> J(jacobian);
        J.topRows<6>().setZero();
        J.bottomRows<1>().setZero();
        return true;
    };
};

/*********** edge *************/
class ReprojectionErrorSE3 : public ceres::SizedCostFunction<2, 7> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ReprojectionErrorSE3(const Eigen::Vector2d& px,
                         const Eigen::Vector3d& wpt,
                         double fx,
                         double fy,
                         double cx,
                         double cy,
                         double sigma = 1)
        : _px(px), _wpt(wpt), _fx(fx), _fy(fy), _cx(cx), _cy(cy) {
        _sqrt_cov = sigma * Eigen::Matrix2d::Identity();
        _sqrt_info = _sqrt_cov.inverse();
    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const override;

    mutable double chi2err;
    mutable bool is_depth_positive;

  private:
    Eigen::Vector2d _px;
    Eigen::Vector3d _wpt;
    Eigen::Matrix2d _sqrt_cov, _sqrt_info;
    double _fx, _fy, _cx, _cy;
};
}  // namespace vo_lab
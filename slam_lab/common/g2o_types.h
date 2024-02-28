#pragma once
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <glog/logging.h>
#include "imu_preintegration.h"
#include "sensor_type.h"

// template arg: minimal dimension of the vertex; type to represent the estimate
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPose() {}
    virtual bool read(std::istream& is) override { return false; }
    virtual bool write(std::ostream& os) const override { return false; }
    // 重置优化变量
    virtual void setToOriginImpl() override { setEstimate(Sophus::SE3d()); }
    // 更新优化变量
    virtual void oplusImpl(const double* update) override {
        auto es = estimate();
        es.so3() = es.so3() * Sophus::SO3d::exp(Eigen::Map<const Eigen::Vector3d>(&update[0]));
        es.translation() += Eigen::Map<const Eigen::Vector3d>(&update[3]);
        setEstimate(es);
    }
};

class VertexVelocity : public g2o::BaseVertex<3, Eigen::Vector3d> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexVelocity() {}
    virtual bool read(std::istream& is) override { return false; }
    virtual bool write(std::ostream& os) const override { return false; }
    virtual void setToOriginImpl() { setEstimate(Eigen::Vector3d::Zero()); }
    virtual void oplusImpl(const double* update) {
        auto es = estimate();
        es += Eigen::Map<const Eigen::Vector3d>(&update[0]);
        setEstimate(es);
    }
};

class VertexGyroBias : public VertexVelocity {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexGyroBias() {}
};

class VertexAcceBias : public VertexVelocity {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexAcceBias() {}
};

// template arg: Dimension of the measurement; type to represent the measurement; vertex type
class EdgeGyroRW : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexGyroBias, VertexGyroBias> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeGyroRW() {}
    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }
    virtual void computeError() override {
        const auto* VG1 = dynamic_cast<const VertexGyroBias*>(_vertices[0]);
        const auto* VG2 = dynamic_cast<const VertexGyroBias*>(_vertices[1]);
        _error = VG2->estimate() - VG1->estimate();
    }
    // 计算雅可比
    virtual void linearizeOplus() override {
        _jacobianOplusXi = -Eigen::Matrix3d::Identity();
        _jacobianOplusXj.setIdentity();
    }
    Eigen::Matrix<double, 6, 6> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }
};

class EdgeAcceRW : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexAcceBias, VertexAcceBias> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeAcceRW() {}
    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }
    virtual void computeError() override {
        const auto* VG1 = dynamic_cast<const VertexAcceBias*>(_vertices[0]);
        const auto* VG2 = dynamic_cast<const VertexAcceBias*>(_vertices[1]);
        _error = VG2->estimate() - VG1->estimate();
    }
    // 计算雅可比
    virtual void linearizeOplus() override {
        _jacobianOplusXi = -Eigen::Matrix3d::Identity();
        _jacobianOplusXj.setIdentity();
    }
    Eigen::Matrix<double, 6, 6> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }
};

class EdgePriorPoseNavState : public g2o::BaseMultiEdge<15, Eigen::Matrix<double, 15, 1>> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePriorPoseNavState(const NavState& measurement, const Eigen::Matrix<double, 15, 15>& info);

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }
    virtual void computeError() override;
    virtual void linearizeOplus() override;

    Eigen::Matrix<double, 15, 15> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 15, 15> J;
        J.block<15, 6>(0, 0) = _jacobianOplus[0];
        J.block<15, 3>(0, 6) = _jacobianOplus[1];
        J.block<15, 3>(0, 9) = _jacobianOplus[2];
        J.block<15, 3>(0, 12) = _jacobianOplus[3];
        return J.transpose() * information() * J;
    }

    NavState _measurement;
};

class EdgeSE3 : public g2o::BaseUnaryEdge<6, Sophus::SE3d, VertexPose> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE3(VertexPose* v,
            const Sophus::SE3d& measurement,
            const Eigen::Matrix<double, 6, 6>& info);

    virtual bool read(std::istream& in) { return false; }
    virtual bool write(std::ostream& out) const { return false; }
    virtual void computeError() override;
    virtual void linearizeOplus() override;

    Eigen::Matrix<double, 6, 6> GetHessian() {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
    }
};

class EdgeInertial : public g2o::BaseMultiEdge<9, Eigen::Matrix<double, 9, 1>> {
  public:
    EdgeInertial(std::shared_ptr<IMUPreintegration> imu_preinteg, const Eigen::Vector3d& gravity)
        : _imu_preinteg(imu_preinteg), _gravity(gravity) {
        resize(6);
        setInformation(_imu_preinteg->GetCov().inverse());
    }

    virtual bool read(std::istream& in) { return false; }
    virtual bool write(std::ostream& out) const { return false; }

    virtual void computeError() override;
    virtual void linearizeOplus() override;

    Eigen::Matrix<double, 24, 24> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 9, 24> J;
        J.block<9, 6>(0, 0) = _jacobianOplus[0];
        J.block<9, 3>(0, 6) = _jacobianOplus[1];
        J.block<9, 3>(0, 9) = _jacobianOplus[2];
        J.block<9, 3>(0, 12) = _jacobianOplus[3];
        J.block<9, 6>(0, 15) = _jacobianOplus[4];
        J.block<9, 3>(0, 21) = _jacobianOplus[5];
        return J.transpose() * information() * J;
    }

  private:
    const std::shared_ptr<IMUPreintegration> _imu_preinteg;
    const Eigen::Vector3d _gravity;
};

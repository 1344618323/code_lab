#include "g2o_types.h"

EdgePriorPoseNavState::EdgePriorPoseNavState(const NavState& measurement,
                                             const Eigen::Matrix<double, 15, 15>& info) {
    resize(4);  // 4 vertices
    _measurement = measurement;
    setInformation(info);
}

void EdgePriorPoseNavState::computeError() {
    auto* vp = dynamic_cast<const VertexPose*>(_vertices[0]);
    auto* vv = dynamic_cast<const VertexVelocity*>(_vertices[1]);
    auto* vbg = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
    auto* vba = dynamic_cast<const VertexAcceBias*>(_vertices[3]);
    const Eigen::Vector3d er = (_measurement._R.inverse() * vp->estimate().so3()).log();
    const Eigen::Vector3d ep = vp->estimate().translation() - _measurement._p;
    const Eigen::Vector3d ev = vv->estimate() - _measurement._v;
    const Eigen::Vector3d ebg = vbg->estimate() - _measurement._bg;
    const Eigen::Vector3d eba = vba->estimate() - _measurement._ba;
    _error << er, ep, ev, ebg, eba;
}

void EdgePriorPoseNavState::linearizeOplus() {
    auto* vp = dynamic_cast<const VertexPose*>(_vertices[0]);
    const Eigen::Vector3d er = (_measurement._R.inverse() * vp->estimate().so3()).log();
    /*
            error(15*1): er ep ev ebg eba
    pose(R,t)
    vel
    bg
    ba
    */
    // de/dpose
    _jacobianOplus[0].setZero();
    _jacobianOplus[0].block<3, 3>(0, 0) = SO3rightJacobianInverse(er);
    _jacobianOplus[0].block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
    // de/dvel
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3, 3>(6, 0) = Eigen::Matrix3d::Identity();
    // de/dbg
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3, 3>(9, 0) = Eigen::Matrix3d::Identity();
    // de/dba
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3, 3>(12, 0) = Eigen::Matrix3d::Identity();
}

EdgeSE3::EdgeSE3(VertexPose* v,
                 const Sophus::SE3d& measurement,
                 const Eigen::Matrix<double, 6, 6>& info) {
    setVertex(0, v);
    setMeasurement(measurement);
    setInformation(info);
}

void EdgeSE3::computeError() {
    auto* vp = dynamic_cast<const VertexPose*>(_vertices[0]);
    const Eigen::Vector3d er = (_measurement.so3().inverse() * vp->estimate().so3()).log();
    const Eigen::Vector3d ep = vp->estimate().translation() - _measurement.translation();
    _error << er, ep;
}

void EdgeSE3::linearizeOplus() {
    auto* vp = dynamic_cast<const VertexPose*>(_vertices[0]);
    const Eigen::Vector3d er = (_measurement.so3().inverse() * vp->estimate().so3()).log();
    _jacobianOplusXi.setZero();
    _jacobianOplusXi.block<3, 3>(0, 0) = SO3rightJacobianInverse(er);
    _jacobianOplusXi.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
}

void EdgeInertial::computeError() {
    auto* p1 = dynamic_cast<const VertexPose*>(_vertices[0]);
    auto* v1 = dynamic_cast<const VertexVelocity*>(_vertices[1]);
    auto* bg1 = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
    auto* ba1 = dynamic_cast<const VertexAcceBias*>(_vertices[3]);
    auto* p2 = dynamic_cast<const VertexPose*>(_vertices[4]);
    auto* v2 = dynamic_cast<const VertexVelocity*>(_vertices[5]);

    Eigen::Vector3d bg = bg1->estimate();
    Eigen::Vector3d ba = ba1->estimate();

    const Sophus::SO3d dR = _imu_preinteg->GetDeltaR(bg);
    const Eigen::Vector3d dv = _imu_preinteg->GetDeltaV(bg, ba);
    const Eigen::Vector3d dp = _imu_preinteg->GetDeltaP(bg, ba);
    double dt = _imu_preinteg->Getdt();

    const Eigen::Vector3d er =
            (dR.inverse() * p1->estimate().so3().inverse() * p2->estimate().so3()).log();
    Eigen::Matrix3d RiT = p1->estimate().so3().inverse().matrix();
    const Eigen::Vector3d ev = RiT * (v2->estimate() - v1->estimate() - _gravity * dt) - dv;
    const Eigen::Vector3d ep = RiT * (p2->estimate().translation() - p1->estimate().translation() -
                                      v1->estimate() * dt - _gravity * dt * dt / 2) -
                               dp;
    _error << er, ev, ep;
}

void EdgeInertial::linearizeOplus() {
    auto* p1 = dynamic_cast<const VertexPose*>(_vertices[0]);
    auto* v1 = dynamic_cast<const VertexVelocity*>(_vertices[1]);
    auto* bg1 = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
    auto* ba1 = dynamic_cast<const VertexAcceBias*>(_vertices[3]);
    auto* p2 = dynamic_cast<const VertexPose*>(_vertices[4]);
    auto* v2 = dynamic_cast<const VertexVelocity*>(_vertices[5]);

    Sophus::SO3d Ri = p1->estimate().so3();
    Sophus::SO3d RiT = Ri.inverse();
    Sophus::SO3d Rj = p2->estimate().so3();
    Eigen::Vector3d vi = v1->estimate();
    Eigen::Vector3d vj = v2->estimate();
    Eigen::Vector3d pi = p1->estimate().translation();
    Eigen::Vector3d pj = p2->estimate().translation();
    Eigen::Vector3d bg = bg1->estimate();
    Eigen::Vector3d ba = ba1->estimate();
    Eigen::Vector3d dbg = bg - _imu_preinteg->GetInitBg();
    auto dR_dbg = _imu_preinteg->GetdRdbg();
    auto dv_dbg = _imu_preinteg->GetdVdbg();
    auto dp_dbg = _imu_preinteg->GetdPdbg();
    auto dv_dba = _imu_preinteg->GetdVdba();
    auto dp_dba = _imu_preinteg->GetdPdba();
    Sophus::SO3d dR = _imu_preinteg->GetDeltaR(bg);
    Sophus::SO3d eR = dR.inverse() * RiT * Rj;
    Eigen::Vector3d er = eR.log();
    Eigen::Matrix3d invJr = SO3rightJacobianInverse(er);
    double dt = _imu_preinteg->Getdt();

    /*
        error: er ev ep
    p1
    v1
    bg1
    ba1
    p2
    v2
    */

    _jacobianOplus[0].setZero();
    // dR/dRi, 4.42
    _jacobianOplus[0].block<3, 3>(0, 0) = -invJr * (Rj.inverse() * Ri).matrix();
    // dv/dRi, 4.47
    _jacobianOplus[0].block<3, 3>(3, 0) = Sophus::SO3d::hat(RiT * (vj - vi - _gravity * dt));
    // dp/dRi, 4.48d
    _jacobianOplus[0].block<3, 3>(6, 0) =
            Sophus::SO3d::hat(RiT * (pj - pi - vi * dt - 0.5 * _gravity * dt * dt));

    /// 残差对p1, 9x3
    // dp/dp1, 4.48a
    _jacobianOplus[0].block<3, 3>(6, 3) = -RiT.matrix();

    /// 残差对v1, 9x3
    _jacobianOplus[1].setZero();
    // dv/dv1, 4.46a
    _jacobianOplus[1].block<3, 3>(3, 0) = -RiT.matrix();
    // dp/dv1, 4.48c
    _jacobianOplus[1].block<3, 3>(6, 0) = -RiT.matrix() * dt;

    /// 残差对bg1
    _jacobianOplus[2].setZero();
    // dR/dbg1, 4.45
    _jacobianOplus[2].block<3, 3>(0, 0) =
            -invJr * eR.inverse().matrix() * SO3rightJacobianInverse((dR_dbg * dbg)) * dR_dbg;
    // dv/dbg1
    _jacobianOplus[2].block<3, 3>(3, 0) = -dv_dbg;
    // dp/dbg1
    _jacobianOplus[2].block<3, 3>(6, 0) = -dp_dbg;

    /// 残差对ba1
    _jacobianOplus[3].setZero();
    // dv/dba1
    _jacobianOplus[3].block<3, 3>(3, 0) = -dv_dba;
    // dp/dba1
    _jacobianOplus[3].block<3, 3>(6, 0) = -dp_dba;

    /// 残差对pose2
    _jacobianOplus[4].setZero();
    // dr/drj, 4.43
    _jacobianOplus[4].block<3, 3>(0, 0) = invJr;
    // dp/dp2, 4.48b
    _jacobianOplus[4].block<3, 3>(6, 3) = RiT.matrix();

    /// 残差对v2
    _jacobianOplus[5].setZero();
    // dv/dv2, 4,46b
    _jacobianOplus[5].block<3, 3>(3, 0) = RiT.matrix();
}

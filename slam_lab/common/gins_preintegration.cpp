#include "gins_preintegration.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

void GinsIntegration::Config(const Option& option) {
    // _option = option;
    // use default value can make a better experiment
    _gnss_info.diagonal() << _option._gnss_ang_var, _option._gnss_ang_var, _option._gnss_ang_var,
            _option._gnss_pose_var, _option._gnss_pose_var, _option._gnss_height_var;
    _gnss_info = _gnss_info.inverse().eval();
    _bg_rw_info.diagonal() << _option._bias_gyro_var, _option._bias_gyro_var,
            _option._bias_gyro_var;
    _bg_rw_info = _bg_rw_info.inverse().eval();
    _ba_rw_info.diagonal() << _option._bias_acce_var, _option._bias_acce_var,
            _option._bias_acce_var;
    _ba_rw_info = _ba_rw_info.inverse().eval();
    _prior_info.block<6, 6>(9, 9) = Eigen::Matrix<double, 6, 6>::Identity() * 1e6;
}

void GinsIntegration::AddIMU(const IMU& imu) {
    if (_imu_preintegration) {
        _imu_preintegration->Integrate(imu, imu._timestamp - _current_time);
    }
    _last_imu = imu;
    _current_time = imu._timestamp;
}

void GinsIntegration::AddGNSS(const GNSS& gnss) {
    if (!gnss._heading_valid) {
        return;
    }
    if (!_imu_preintegration) {
        // receive the first gnss
        _cur_state = NavState(gnss._timestamp,
                              gnss._utm_pose.translation(),
                              gnss._utm_pose.so3(),
                              Eigen::Vector3d::Zero(),
                              _option._imu_preintegration_option.init_bg,
                              _option._imu_preintegration_option.init_ba,
                              _option._gravity);
        _imu_preintegration =
                std::make_shared<IMUPreintegration>(_option._imu_preintegration_option);
        _cur_gnss = gnss;
    } else {
        _imu_preintegration->Integrate(_last_imu, gnss._timestamp - _current_time);
        _cur_state = _imu_preintegration->Predict(_last_state);
        _cur_gnss = gnss;
        Optimize();
    }

    _last_gnss = _cur_gnss;
    _last_state = _cur_state;
    _current_time = gnss._timestamp;
}

void GinsIntegration::Optimize() {
    if (_imu_preintegration->Getdt() < 1e-3) {
        return;
    }

    using BlockSolverType = g2o::BlockSolverX;
    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

    auto* solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    auto v0_pose = new VertexPose();
    v0_pose->setId(0);
    v0_pose->setEstimate(_last_state.GetSE3());
    optimizer.addVertex(v0_pose);

    auto v0_vel = new VertexVelocity();
    v0_vel->setId(1);
    v0_vel->setEstimate(_last_state._v);
    optimizer.addVertex(v0_vel);

    auto v0_bg = new VertexGyroBias();
    v0_bg->setId(2);
    v0_bg->setEstimate(_last_state._bg);
    optimizer.addVertex(v0_bg);

    auto v0_ba = new VertexAcceBias();
    v0_ba->setId(3);
    v0_ba->setEstimate(_last_state._ba);
    optimizer.addVertex(v0_ba);

    auto v1_pose = new VertexPose();
    v1_pose->setId(4);
    v1_pose->setEstimate(_cur_state.GetSE3());
    optimizer.addVertex(v1_pose);

    auto v1_vel = new VertexVelocity();
    v1_vel->setId(5);
    v1_vel->setEstimate(_cur_state._v);
    optimizer.addVertex(v1_vel);

    auto v1_bg = new VertexGyroBias();
    v1_bg->setId(6);
    v1_bg->setEstimate(_cur_state._bg);
    optimizer.addVertex(v1_bg);

    auto v1_ba = new VertexAcceBias();
    v1_ba->setId(7);
    v1_ba->setEstimate(_cur_state._ba);
    optimizer.addVertex(v1_ba);

    auto edge_inertial = new EdgeInertial(_imu_preintegration, _option._gravity);
    edge_inertial->setVertex(0, v0_pose);
    edge_inertial->setVertex(1, v0_vel);
    edge_inertial->setVertex(2, v0_bg);
    edge_inertial->setVertex(3, v0_ba);
    edge_inertial->setVertex(4, v1_pose);
    edge_inertial->setVertex(5, v1_vel);
    optimizer.addEdge(edge_inertial);

    auto* edge_gyro_rw = new EdgeGyroRW();
    edge_gyro_rw->setVertex(0, v0_bg);
    edge_gyro_rw->setVertex(1, v1_bg);
    edge_gyro_rw->setInformation(_bg_rw_info);
    optimizer.addEdge(edge_gyro_rw);

    auto* edge_acce_rw = new EdgeAcceRW();
    edge_acce_rw->setVertex(0, v0_ba);
    edge_acce_rw->setVertex(1, v1_ba);
    edge_acce_rw->setInformation(_ba_rw_info);
    optimizer.addEdge(edge_acce_rw);

    auto edge_gnss0 = new EdgeSE3(v0_pose, _last_gnss._utm_pose, _gnss_info);
    optimizer.addEdge(edge_gnss0);

    auto edge_gnss1 = new EdgeSE3(v1_pose, _cur_gnss._utm_pose, _gnss_info);
    optimizer.addEdge(edge_gnss1);

    auto* edge_prior = new EdgePriorPoseNavState(_last_state, _prior_info);
    edge_prior->setVertex(0, v0_pose);
    edge_prior->setVertex(1, v0_vel);
    edge_prior->setVertex(2, v0_bg);
    edge_prior->setVertex(3, v0_ba);
    optimizer.addEdge(edge_prior);

    optimizer.setVerbose(_option._verbose);
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    if (_option._verbose) {
        LOG(INFO) << "chi2/error: ";
        LOG(INFO) << "preintegration: " << edge_inertial->chi2() << "/"
                  << edge_inertial->error().transpose();
        LOG(INFO) << "gnss0: " << edge_gnss0->chi2() << "/ " << edge_gnss0->error().transpose();
        LOG(INFO) << "gnss1: " << edge_gnss1->chi2() << "/ " << edge_gnss1->error().transpose();
        LOG(INFO) << "gyro bias: " << edge_gyro_rw->chi2() << "/"
                  << edge_gyro_rw->error().transpose();
        LOG(INFO) << "acce bias: " << edge_acce_rw->chi2() << "/"
                  << edge_acce_rw->error().transpose();
        LOG(INFO) << "prior: " << edge_prior->chi2() << "/" << edge_prior->error().transpose();
    }

    _last_state._R = v0_pose->estimate().so3();
    _last_state._p = v0_pose->estimate().translation();
    _last_state._v = v0_vel->estimate();
    _last_state._bg = v0_bg->estimate();
    _last_state._ba = v0_ba->estimate();
    _cur_state._R = v1_pose->estimate().so3();
    _cur_state._p = v1_pose->estimate().translation();
    _cur_state._v = v1_vel->estimate();
    _cur_state._bg = v1_bg->estimate();
    _cur_state._ba = v1_ba->estimate();

    IMUPreintegration::Option option = _option._imu_preintegration_option;
    option.init_bg = _cur_state._bg;
    option.init_ba = _cur_state._ba;
    _imu_preintegration = std::make_shared<IMUPreintegration>(option);
}

NavState GinsIntegration::GetState() const {
    if (!_imu_preintegration) {
        return {};
    }
    return _imu_preintegration->Predict(_cur_state);
}

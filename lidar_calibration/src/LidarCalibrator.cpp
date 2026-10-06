#include <lidar_calibration/LidarCalibrator.hpp>

Calibrator::Calibrator(Config                           cfg,
                       std::map<SensorId, gtsam::Pose3> extrinsics_init)
    : cfg_(cfg)
    , processor_(cfg.processor)
    , problem_(CalibrationProblem::Config{cfg.reference_id, cfg.max_corr_dist, cfg.max_outer_iter,
                                          cfg.twist_tol, cfg.extrinsic_tol})
    , extrinsics_(std::move(extrinsics_init))
    , twists_() {}

void Calibrator::addScan(const SensorId& id, Timestamp timestamp,
                         pcl::PointCloud<pcl::PointXYZ> cloud) {
    window_.insertLidarMeasurement(id, timestamp, std::move(cloud));
}

CalibrationResult Calibrator::solve() {
    const auto& ref = window_.lidarChannel(cfg_.reference_id).series;
    const std::size_t needed = static_cast<std::size_t>(cfg_.chain_length) + 1;
    if (ref.size() < needed) return {};

    twists_.resize(ref.size() - 1, gtsam::Vector6::Zero());

    const auto result = problem_.solve(window_, processor_, extrinsics_, twists_);

    extrinsics_ = result.extrinsics;
    twists_     = result.twists;

    return result;
}

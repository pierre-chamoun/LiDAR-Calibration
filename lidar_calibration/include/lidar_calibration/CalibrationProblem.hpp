#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

#include <lidar_calibration/LidarProcessor.hpp>
#include <lidar_calibration/Sensors.hpp>
#include <lidar_calibration/factors.hpp>
#include <map>
#include <utility>
#include <vector>

struct CalibrationResult {
    std::map<SensorId, gtsam::Pose3> extrinsics;
    std::vector<gtsam::Vector6> twists;
    std::map<SensorId, gtsam::Matrix66> cov_extrinsics;
    std::vector<gtsam::Matrix66> cov_twists;
    int outer_iterations = 0;
    bool twist_converged = false;
    bool extrinsic_converged = false;
};

class CalibrationProblem {
   public:
    struct Config {
        SensorId reference_id;
        double max_corr_dist = 0.3;
        int max_outer_iter = 5;
        double twist_tol = 1e-4;
        double extrinsic_tol = 1e-4;
    };

    explicit CalibrationProblem(Config cfg);

    CalibrationResult solve(const MultiSensorChannel& window, LidarProcessor& processor,
                            std::map<SensorId, gtsam::Pose3> extrinsics_init,
                            std::vector<gtsam::Vector6> twists_init) const;

   private:
    Config cfg_;

    static gtsam::SharedNoiseModel gicpNoise(const Eigen::Matrix3d& covariance_source,
                                             const Eigen::Matrix3d& covariance_target,
                                             const gtsam::Rot3& rotation);

    std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> buildGraph(
        const MultiSensorChannel& window, LidarProcessor& processor,
        const std::map<SensorId, gtsam::Pose3>& extrinsics,
        const std::vector<gtsam::Vector6>& twists, bool& any_f1) const;

    int addScanToScanFactors(gtsam::NonlinearFactorGraph& graph,
                             const ProcessedLidarMeasurement& source,
                             const ProcessedLidarMeasurement& target, double T,
                             const gtsam::Vector6& twist, gtsam::Key key_twist, float max_d2) const;

    int addCrossLidarFactors(gtsam::NonlinearFactorGraph& graph,
                             const ProcessedLidarMeasurement& source,
                             const ProcessedLidarMeasurement& target, double dt,
                             const gtsam::Vector6& twist, gtsam::Key key_twist, gtsam::Key key_e,
                             const gtsam::Pose3& T_ref_other, float max_d2) const;
};

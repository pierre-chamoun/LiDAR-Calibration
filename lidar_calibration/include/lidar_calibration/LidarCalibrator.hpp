#pragma once

#include <functional>
#include <lidar_calibration/CalibrationProblem.hpp>
#include <lidar_calibration/LidarProcessor.hpp>
#include <lidar_calibration/Sensors.hpp>
#include <map>
#include <string>

class Calibrator {
   public:
    struct Config {
        SensorId reference_id;
        LidarProcessor::Config processor;
        double max_corr_dist = 0.3;
        int max_outer_iter = 5;
        int chain_length = 3;
        double twist_tol = 1e-4;
        double extrinsic_tol = 1e-4;
    };

    Calibrator(Config cfg, std::map<SensorId, gtsam::Pose3> extrinsics_init);

    void addScan(const SensorId& id, Timestamp timestamp, pcl::PointCloud<pcl::PointXYZ> cloud);

    CalibrationResult solve();

    const std::map<SensorId, gtsam::Pose3>& extrinsics() const { return extrinsics_; }
    const std::vector<gtsam::Vector6>& twists() const { return twists_; }

   private:
    Config cfg_;
    MultiSensorChannel window_;
    LidarProcessor processor_;
    CalibrationProblem problem_;
    std::map<SensorId, gtsam::Pose3> extrinsics_;
    std::vector<gtsam::Vector6> twists_;
};

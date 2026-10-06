#pragma once

#include <gtsam/geometry/Pose3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <random>
#include <vector>

pcl::PointCloud<pcl::PointXYZ> perturbCloud(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::vector<std::pair<double, double>>& noise_sigma_r_bins,
    double sigma_phi,
    double sigma_theta,
    std::mt19937& rng);

struct BatchEstimate {
    gtsam::Pose3 T_est;
    Eigen::Matrix<double, 6, 6> cov;
    Eigen::Matrix<double, 6, 6> info;
    int n_bootstrap;
};

BatchEstimate makeBatchEstimate(const gtsam::Pose3& T_clean,
                                const std::vector<gtsam::Pose3>& perturbed_estimates);

gtsam::Pose3 infoWeightedMean(const std::vector<BatchEstimate>& estimates,
                               gtsam::Pose3 T_init,
                               int max_iter = 100);

struct FusionResult {
    gtsam::Pose3 T_fused;
    Eigen::Matrix<double, 6, 6> cov_independent;
    Eigen::Matrix<double, 6, 6> cov_sandwich;
};

FusionResult fuseBatchEstimates(const std::vector<BatchEstimate>& estimates,
                                 gtsam::Pose3 T_init);

Eigen::Matrix<double, 6, 6> transportLeftCovariance(const Eigen::Matrix<double, 6, 6>& Sigma_d,
                                                    const gtsam::Pose3& G);

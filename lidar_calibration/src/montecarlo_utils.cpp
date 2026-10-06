#include <cmath>
#include <lidar_calibration/montecarlo_utils.hpp>

pcl::PointCloud<pcl::PointXYZ> perturbCloud(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::vector<std::pair<double, double>>& noise_sigma_r_bins, double sigma_phi,
    double sigma_theta, std::mt19937& rng) {
    std::normal_distribution<double> nphi(0.0, sigma_phi);
    std::normal_distribution<double> ntheta(0.0, sigma_theta);

    pcl::PointCloud<pcl::PointXYZ> out = cloud;
    for (auto& pt : out.points) {
        const double x = pt.x, y = pt.y, z = pt.z;
        const double r = std::sqrt(x * x + y * y + z * z);
        if (r < 1e-6) continue;

        double sigma_r = noise_sigma_r_bins.back().second;
        for (const auto& [max_r, sig] : noise_sigma_r_bins)
            if (r < max_r) {
                sigma_r = sig;
                break;
            }
        std::normal_distribution<double> nr(0.0, sigma_r);

        const double phi = std::atan2(y, x);
        const double theta = std::acos(std::clamp(z / r, -1.0, 1.0));
        const double rp = r + nr(rng);
        const double phi_p = phi + nphi(rng);
        const double tht_p = theta + ntheta(rng);

        pt.x = static_cast<float>(rp * std::sin(tht_p) * std::cos(phi_p));
        pt.y = static_cast<float>(rp * std::sin(tht_p) * std::sin(phi_p));
        pt.z = static_cast<float>(rp * std::cos(tht_p));
    }
    return out;
}

BatchEstimate makeBatchEstimate(const gtsam::Pose3& T_clean,
                                const std::vector<gtsam::Pose3>& perturbed_estimates) {
    const int M = static_cast<int>(perturbed_estimates.size());

    // Left-tangent residuals around T_clean: xi_i = log(T_i * T_clean^{-1})
    Eigen::Matrix<double, 6, 6> C = Eigen::Matrix<double, 6, 6>::Zero();
    for (const auto& Ti : perturbed_estimates) {
        gtsam::Vector6 xi = gtsam::Pose3::Logmap(Ti.compose(T_clean.inverse()));
        C += xi * xi.transpose();
    }
    C /= static_cast<double>(M - 1);  // Bessel correction
    C += 1e-9 * Eigen::Matrix<double, 6, 6>::Identity();

    Eigen::Matrix<double, 6, 6> info = C.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

    return BatchEstimate{T_clean, C, info, M};
}

gtsam::Pose3 infoWeightedMean(const std::vector<BatchEstimate>& estimates, gtsam::Pose3 T,
                              int max_iter) {
    for (int iter = 0; iter < max_iter; ++iter) {
        Eigen::Matrix<double, 6, 6> H_sum = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> Hb = Eigen::Matrix<double, 6, 1>::Zero();
        for (const auto& e : estimates) {
            gtsam::Vector6 xi = gtsam::Pose3::Logmap(e.T_est.compose(T.inverse()));
            H_sum += e.info;
            Hb += e.info * xi;
        }
        gtsam::Vector6 delta = H_sum.ldlt().solve(Hb);
        T = gtsam::Pose3::Expmap(delta).compose(T);
        if (delta.norm() < 1e-8) break;
    }
    return T;
}

FusionResult fuseBatchEstimates(const std::vector<BatchEstimate>& estimates, gtsam::Pose3 T_init) {
    const gtsam::Pose3 T_fused = infoWeightedMean(estimates, T_init);

    Eigen::Matrix<double, 6, 6> H_sum = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> meat = Eigen::Matrix<double, 6, 6>::Zero();
    for (const auto& e : estimates) {
        gtsam::Vector6 r = gtsam::Pose3::Logmap(e.T_est.compose(T_fused.inverse()));
        H_sum += e.info;
        meat += e.info * r * r.transpose() * e.info;
    }

    Eigen::Matrix<double, 6, 6> B_inv = H_sum.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

    return FusionResult{T_fused, B_inv, B_inv * meat * B_inv};
}

Eigen::Matrix<double, 6, 6> transportLeftCovariance(const Eigen::Matrix<double, 6, 6>& Sigma_d,
                                                    const gtsam::Pose3& G) {
    const gtsam::Matrix66 J = gtsam::Pose3::LogmapDerivative(G) * G.inverse().AdjointMap();
    return J * Sigma_d * J.transpose();
}

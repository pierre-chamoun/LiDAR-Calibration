#include <lidar_calibration/CalibrationProblem.hpp>

namespace {
gtsam::Key extrinsicKey(std::size_t i) { return gtsam::Symbol('e', i); }
gtsam::Key twistKey(std::size_t k) { return gtsam::Symbol('w', k); }

std::vector<SensorId> otherIds(const MultiSensorChannel& window, const SensorId& reference_id) {
    std::vector<SensorId> ids;
    for (const auto& id : window.lidarIds()) {
        if (id == reference_id) continue;
        ids.push_back(id);
    }
    return ids;
}
}  // namespace

CalibrationProblem::CalibrationProblem(Config cfg) : cfg_(std::move(cfg)) {}

gtsam::SharedNoiseModel CalibrationProblem::gicpNoise(const Eigen::Matrix3d& covariance_source,
                                                      const Eigen::Matrix3d& covariance_target,
                                                      const gtsam::Rot3& rotation) {
    const Eigen::Matrix3d R = rotation.matrix();
    Eigen::Matrix3d C = covariance_target + R * covariance_source * R.transpose();
    C.diagonal().array() += 1e-6;
    return gtsam::noiseModel::Gaussian::Information(C.inverse());
}

int CalibrationProblem::addScanToScanFactors(gtsam::NonlinearFactorGraph& graph,
                                             const ProcessedLidarMeasurement& source,
                                             const ProcessedLidarMeasurement& target, double T,
                                             const gtsam::Vector6& twist, gtsam::Key key_twist,
                                             float max_d2) const {
    int n_f1 = 0;

    const gtsam::Pose3 transformation = gtsam::Pose3::Expmap(twist * T);

    pcl::PointCloud<pcl::PointXYZ> transformed_source;
    pcl::transformPointCloud(*source.cloud, transformed_source,
                             transformation.matrix().cast<float>());

    std::vector<int> idx(1);
    std::vector<float> dist(1);

    for (std::size_t i = 0; i < source.cloud->size(); ++i) {
        if (target.tree->nearestKSearch(transformed_source[i], 1, idx, dist) > 0 &&
            dist[0] <= max_d2) {
            const int j = idx[0];
            graph.add(ScanToScanFactor(key_twist,
                                       (*source.cloud)[i].getVector3fMap().cast<double>(),
                                       (*target.cloud)[j].getVector3fMap().cast<double>(), T,
                                       gicpNoise(target.covariances[j], source.covariances[i],
                                                 transformation.rotation())));
            ++n_f1;
        }
    }

    return n_f1;
}

int CalibrationProblem::addCrossLidarFactors(gtsam::NonlinearFactorGraph& graph,
                                             const ProcessedLidarMeasurement& source,
                                             const ProcessedLidarMeasurement& target, double dt,
                                             const gtsam::Vector6& twist, gtsam::Key key_twist,
                                             gtsam::Key key_e, const gtsam::Pose3& T_ref_other,
                                             float max_d2) const {
    int n_f2 = 0;

    const gtsam::Pose3 transformation = gtsam::Pose3::Expmap(twist * dt) * T_ref_other;

    pcl::PointCloud<pcl::PointXYZ> transformed_source;
    pcl::transformPointCloud(*source.cloud, transformed_source,
                             transformation.matrix().cast<float>());

    std::vector<int> idx(1);
    std::vector<float> dist(1);

    for (std::size_t i = 0; i < source.cloud->size(); ++i) {
        if (target.tree->nearestKSearch(transformed_source[i], 1, idx, dist) > 0 &&
            dist[0] <= max_d2) {
            const int j = idx[0];
            graph.add(CrossLidarFactor(key_twist, key_e,
                                       (*source.cloud)[i].getVector3fMap().cast<double>(),
                                       (*target.cloud)[j].getVector3fMap().cast<double>(), dt,
                                       gicpNoise(target.covariances[j], source.covariances[i],
                                                 transformation.rotation())));
            ++n_f2;
        }
    }

    if (n_f2 < 20)
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(
            key_e, T_ref_other,
            gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished())));

    return n_f2;
}

std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> CalibrationProblem::buildGraph(
    const MultiSensorChannel& window, LidarProcessor& processor,
    const std::map<SensorId, gtsam::Pose3>& extrinsics, const std::vector<gtsam::Vector6>& twists,
    bool& any_f1) const {
    const auto& ref_channel = window.lidarChannel(cfg_.reference_id);
    const auto& ref_stream = ref_channel.series;
    const std::size_t N = ref_stream.size() - 1;
    const float max_d2 = static_cast<float>(cfg_.max_corr_dist * cfg_.max_corr_dist);
    const std::vector<SensorId> other_ids = otherIds(window, cfg_.reference_id);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;

    for (std::size_t k = 0; k < N; ++k) values.insert(twistKey(k), twists[k]);
    for (std::size_t i = 0; i < other_ids.size(); ++i)
        values.insert(extrinsicKey(i), extrinsics.at(other_ids[i]));

    any_f1 = false;

    std::size_t k = 0;
    for (auto it = ref_stream.begin(); std::next(it) != ref_stream.end(); ++it, ++k) {
        const auto& prev_entry = *it;
        const auto& curr_entry = *std::next(it);
        const Timestamp t_curr = curr_entry.timestamp;

        const double T = static_cast<double>(t_curr - prev_entry.timestamp) * 1e-9;

        const auto* ref_prev = processor.get(ref_channel, prev_entry.timestamp);
        const auto* ref_curr = processor.get(ref_channel, t_curr);
        if (!ref_prev || !ref_curr) continue;

        const int n_f1 =
            addScanToScanFactors(graph, *ref_prev, *ref_curr, T, twists[k], twistKey(k), max_d2);
        if (n_f1 > 0) any_f1 = true;

        for (std::size_t i = 0; i < other_ids.size(); ++i) {
            const auto& other_channel = window.lidarChannel(other_ids[i]);
            const gtsam::Pose3& T_ref_other = extrinsics.at(other_channel.id);

            const auto* other_entry = other_channel.series.closestTo(t_curr);
            const auto* other_curr =
                other_entry ? processor.get(other_channel, other_entry->timestamp) : nullptr;

            if (other_curr) {
                const double dt = static_cast<double>(t_curr - other_entry->timestamp) * 1e-9;
                addCrossLidarFactors(graph, *other_curr, *ref_curr, dt, twists[k], twistKey(k),
                                     extrinsicKey(i), T_ref_other, max_d2);
            } else {
                graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                    extrinsicKey(i), T_ref_other,
                    gtsam::noiseModel::Diagonal::Sigmas(
                        (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished())));
            }
        }
    }

    return {std::move(graph), std::move(values)};
}

CalibrationResult CalibrationProblem::solve(const MultiSensorChannel& window,
                                            LidarProcessor& processor,
                                            std::map<SensorId, gtsam::Pose3> extrinsics,
                                            std::vector<gtsam::Vector6> twists) const {
    const auto& ref_channel = window.lidarChannel(cfg_.reference_id);
    const auto& ref_stream = ref_channel.series;

    if (ref_stream.size() < 2) return {};
    const std::size_t N = ref_stream.size() - 1;

    if (twists.size() != N) twists.assign(N, gtsam::Vector6::Zero());

    const std::vector<SensorId> other_ids = otherIds(window, cfg_.reference_id);

    {
        for (const auto& entry : ref_stream) processor.get(ref_channel, entry.timestamp);
        for (const auto& other_id : other_ids) {
            const auto& other_ch = window.lidarChannel(other_id);
            auto it = ref_stream.begin() + 1;
            for (; it != ref_stream.end(); ++it) {
                const auto* e = other_ch.series.closestTo(it->timestamp);
                if (e) processor.get(other_ch, e->timestamp);
            }
        }
    }

    gtsam::LevenbergMarquardtParams lm;
    lm.setVerbosity("SILENT");

    std::map<SensorId, gtsam::Matrix66> cov_extrinsics;
    std::vector<gtsam::Matrix66> cov_twists(N, gtsam::Matrix66::Zero());
    for (const auto& id : other_ids) cov_extrinsics[id] = gtsam::Matrix66::Zero();

    int outer_iterations = 0;
    bool twist_converged = false;
    bool extrinsic_converged = false;

    for (int outer = 0; outer < cfg_.max_outer_iter; ++outer) {
        outer_iterations = outer + 1;

        bool any_f1 = false;
        auto [graph, values] = buildGraph(window, processor, extrinsics, twists, any_f1);
        if (!any_f1) break;

        const auto result = gtsam::LevenbergMarquardtOptimizer(graph, values, lm).optimize();

        double delta_twist = 0.0;
        for (std::size_t k = 0; k < N; ++k) {
            const auto new_twist = result.at<gtsam::Vector6>(twistKey(k));
            delta_twist = std::max(delta_twist, (new_twist - twists[k]).norm());
            twists[k] = new_twist;
        }
        double delta_extrinsic = 0.0;
        for (std::size_t i = 0; i < other_ids.size(); ++i) {
            const auto new_extrinsic = result.at<gtsam::Pose3>(extrinsicKey(i));
            delta_extrinsic = std::max(
                delta_extrinsic, extrinsics[other_ids[i]].localCoordinates(new_extrinsic).norm());
            extrinsics[other_ids[i]] = new_extrinsic;
        }

        twist_converged = delta_twist < cfg_.twist_tol;
        extrinsic_converged = delta_extrinsic < cfg_.extrinsic_tol;
        const bool converged = twist_converged && extrinsic_converged;
        if (converged || outer == cfg_.max_outer_iter - 1) {
            gtsam::Marginals marginals(graph, result);
            for (std::size_t i = 0; i < other_ids.size(); ++i)
                cov_extrinsics[other_ids[i]] = marginals.marginalCovariance(extrinsicKey(i));
            for (std::size_t k = 0; k < N; ++k)
                cov_twists[k] = marginals.marginalCovariance(twistKey(k));
        }
        if (converged) break;
    }

    return CalibrationResult{extrinsics,       twists,          cov_extrinsics,     cov_twists,
                             outer_iterations, twist_converged, extrinsic_converged};
}

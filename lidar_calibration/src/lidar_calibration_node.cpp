#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <lidar_calibration/LidarCalibrator.hpp>
#include <lidar_calibration/montecarlo_utils.hpp>
#include <optional>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// ─────────────────────────────────────────────────────────────────────────────

class LidarCalibrationNode : public rclcpp::Node {
   public:
    LidarCalibrationNode() : Node("lidar_calibration_node") {
        getParameters();
        if (bag_path_.empty()) {
            RCLCPP_ERROR(get_logger(), "bag_path parameter is required");
            return;
        }
        run();
    }

   private:
    std::string bag_path_, source_topic_, target_topic_, output_dir_;

    int batch_stride_{0};
    int batch_count_{-1};  // -1 = unlimited
    long int start_offset_scans_{0};

    int bootstrap_n_{20};
    std::vector<std::pair<double, double>> noise_sigma_r_bins_;
    double noise_sigma_phi_{0.001};
    double noise_sigma_theta_{0.001};
    std::mt19937 rng_{std::random_device{}()};

    Calibrator::Config calibrator_cfg_;
    gtsam::Pose3 extrinsic_init_;

    void getParameters() {
        bag_path_ = declare_parameter<std::string>("bag_path", "");
        source_topic_ = declare_parameter<std::string>("source_topic", "/ouster0");
        target_topic_ = declare_parameter<std::string>("target_topic", "/ouster1");
        output_dir_ = declare_parameter<std::string>("output_dir", "output/calibration");
        batch_stride_ = declare_parameter<int>("preprocessing.batch_stride", 0);
        batch_count_ = declare_parameter<int>("preprocessing.batch_count", -1);
        start_offset_scans_ = declare_parameter<int>("preprocessing.start_offset_scans", 0);

        const double tx = declare_parameter<double>("initial_guess.tx", 0.0);
        const double ty = declare_parameter<double>("initial_guess.ty", 0.0);
        const double tz = declare_parameter<double>("initial_guess.tz", 0.0);
        const double roll = declare_parameter<double>("initial_guess.roll", 0.0);
        const double pitch = declare_parameter<double>("initial_guess.pitch", 0.0);
        const double yaw = declare_parameter<double>("initial_guess.yaw", 0.0);

        calibrator_cfg_.reference_id = source_topic_;
        calibrator_cfg_.processor.voxel_size =
            static_cast<float>(declare_parameter<double>("preprocessing.voxel_size", 0.2));
        calibrator_cfg_.processor.k_neighbors = declare_parameter<int>("gicp.k_neighbors", 25);
        calibrator_cfg_.max_corr_dist =
            declare_parameter<double>("gicp.max_correspondence_distance_voxels", 1.0) *
            static_cast<double>(calibrator_cfg_.processor.voxel_size);
        calibrator_cfg_.max_outer_iter = declare_parameter<int>("gicp.max_iterations", 5);
        calibrator_cfg_.chain_length = declare_parameter<int>("gicp.chain_length", 3);
        calibrator_cfg_.twist_tol = declare_parameter<double>("gicp.twist_tolerance", 1e-4);
        calibrator_cfg_.extrinsic_tol = declare_parameter<double>("gicp.extrinsic_tolerance", 1e-4);

        const gtsam::Rot3 R =
            gtsam::Rot3::RzRyRx(roll * M_PI / 180.0, pitch * M_PI / 180.0, yaw * M_PI / 180.0);
        extrinsic_init_ = gtsam::Pose3(R, gtsam::Point3(tx, ty, tz));

        bootstrap_n_ = declare_parameter<int>("bootstrap.n", 20);
        {
            auto max_ranges =
                declare_parameter<std::vector<double>>("bootstrap.noise_sigma_r_bins.max_range_m",
                                                       std::vector<double>{1.0, 20.0, 50.0, 1.0e9});
            auto sigmas = declare_parameter<std::vector<double>>(
                "bootstrap.noise_sigma_r_bins.sigma_r_m",
                std::vector<double>{0.007, 0.010, 0.020, 0.050});
            for (std::size_t i = 0; i < std::min(max_ranges.size(), sigmas.size()); ++i)
                noise_sigma_r_bins_.emplace_back(max_ranges[i], sigmas[i]);
        }
        noise_sigma_phi_ =
            declare_parameter<double>("bootstrap.noise_sigma_phi_deg", 0.0586) * M_PI / 180.0;
        noise_sigma_theta_ =
            declare_parameter<double>("bootstrap.noise_sigma_theta_deg", 0.234) * M_PI / 180.0;
    }

    struct RawScan {
        Timestamp ts;
        pcl::PointCloud<pcl::PointXYZ> cloud;
    };

    template <typename ToCloud>
    std::unique_ptr<Calibrator> buildCalibrator(const std::vector<RawScan>& src_scans,
                                                const std::vector<RawScan>& tgt_scans,
                                                ToCloud&& to_cloud) const {
        auto cal = std::make_unique<Calibrator>(
            calibrator_cfg_, std::map<SensorId, gtsam::Pose3>{{target_topic_, extrinsic_init_}});
        for (const auto& s : src_scans) cal->addScan(source_topic_, s.ts, to_cloud(s.cloud));
        for (const auto& s : tgt_scans) cal->addScan(target_topic_, s.ts, to_cloud(s.cloud));
        return cal;
    }

    std::vector<gtsam::Pose3> runBootstrap(const std::vector<RawScan>& src_scans,
                                           const std::vector<RawScan>& tgt_scans) {
        std::vector<gtsam::Pose3> perturbed;
        perturbed.reserve(bootstrap_n_);

        for (int b = 0; b < bootstrap_n_; ++b) {
            auto cal =
                buildCalibrator(src_scans, tgt_scans, [&](const pcl::PointCloud<pcl::PointXYZ>& c) {
                    return perturbCloud(c, noise_sigma_r_bins_, noise_sigma_phi_,
                                        noise_sigma_theta_, rng_);
                });
            const auto r = cal->solve();
            if (r.extrinsics.empty()) continue;
            if (!(r.twist_converged && r.extrinsic_converged)) continue;
            perturbed.push_back(r.extrinsics.at(target_topic_));
        }
        return perturbed;
    }

    void logBatch(const CalibrationResult& r_clean, const BatchEstimate& est, long int scan_idx,
                  int N, std::ofstream& csv) {
        const auto& T_e = r_clean.extrinsics.at(target_topic_);
        const Eigen::Matrix<double, 6, 6>& C = est.cov;

        csv << r_clean.outer_iterations << "," << (r_clean.twist_converged ? 1 : 0) << ","
            << (r_clean.extrinsic_converged ? 1 : 0) << "," << scan_idx << ","
            << T_e.translation().x() << "," << T_e.translation().y() << "," << T_e.translation().z()
            << "," << T_e.rotation().roll() * 180.0 / M_PI << ","
            << T_e.rotation().pitch() * 180.0 / M_PI << "," << T_e.rotation().yaw() * 180.0 / M_PI
            << "," << 3.0 * std::sqrt(C(0, 0)) * 180.0 / M_PI << ","
            << 3.0 * std::sqrt(C(1, 1)) * 180.0 / M_PI << ","
            << 3.0 * std::sqrt(C(2, 2)) * 180.0 / M_PI << "," << 3.0 * std::sqrt(C(3, 3)) << ","
            << 3.0 * std::sqrt(C(4, 4)) << "," << 3.0 * std::sqrt(C(5, 5)) << ","
            << est.n_bootstrap;

        for (int k = 0; k < N; ++k) {
            const auto& w = r_clean.twists[k];
            csv << "," << w(0) << "," << w(1) << "," << w(2) << "," << w(3) << "," << w(4) << ","
                << w(5);
        }
        csv << "\n";
        csv.flush();

        RCLCPP_INFO(get_logger(),
                    "Batch %ld  T t=(%.4f, %.4f, %.4f)  r=(%.3f, %.3f, %.3f) deg  "
                    "3sig_t=(%.4f, %.4f, %.4f) m  bootstrap=%d",
                    scan_idx, T_e.translation().x(), T_e.translation().y(), T_e.translation().z(),
                    T_e.rotation().roll() * 180.0 / M_PI, T_e.rotation().pitch() * 180.0 / M_PI,
                    T_e.rotation().yaw() * 180.0 / M_PI, 3.0 * std::sqrt(C(3, 3)),
                    3.0 * std::sqrt(C(4, 4)), 3.0 * std::sqrt(C(5, 5)), est.n_bootstrap);
    }

    void run() {
        std::filesystem::create_directories(output_dir_);

        const int N = calibrator_cfg_.chain_length;
        const int BATCH = N + 1;

        std::ofstream csv(output_dir_ + "/calibration.csv");
        csv << std::fixed << std::setprecision(6)
            << "outer_iterations,twist_converged,extrinsic_converged,"
            << "scan_idx,T_x,T_y,T_z,T_roll_deg,T_pitch_deg,T_yaw_deg,"
            << "3sig_ox,3sig_oy,3sig_oz,3sig_x,3sig_y,3sig_z,n_bootstrap";
        for (int k = 1; k <= N; ++k)
            csv << ",w" << k << "_ox,w" << k << "_oy,w" << k << "_oz"
                << ",w" << k << "_vx,w" << k << "_vy,w" << k << "_vz";
        csv << "\n";

        const std::string dataset_name = std::filesystem::path(output_dir_).filename().string();
        std::ofstream rec(output_dir_ + "/recursive_estimate_" + dataset_name + ".csv");
        rec << std::fixed << std::setprecision(9) << "n,roll_deg,pitch_deg,yaw_deg,tx,ty,tz,"
            << "c00,c01,c02,c03,c04,c05,c11,c12,c13,c14,c15,c22,c23,c24,c25,"
            << "c33,c34,c35,c44,c45,c55,n_inliers\n";

        rosbag2_cpp::Reader reader;
        reader.open(bag_path_);
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> ser;

        const int period = std::max(batch_stride_, BATCH);

        std::vector<RawScan> src_scans, tgt_scans;
        long int src_raw_seen = 0;
        long int src_seen = 0;
        int in_window_count = 0;
        long int batch_idx = 0;
        long int batch_start_idx = 0;
        std::vector<BatchEstimate> batch_estimates;

        while (reader.has_next()) {
            if (batch_count_ >= 0 && batch_idx >= batch_count_) break;

            auto msg = reader.read_next();
            if (msg->topic_name != source_topic_ && msg->topic_name != target_topic_) continue;

            if (msg->topic_name == source_topic_ && src_raw_seen < start_offset_scans_) {
                ++src_raw_seen;
                continue;
            }

            rclcpp::SerializedMessage sm(*msg->serialized_data);
            sensor_msgs::msg::PointCloud2 ros_msg;
            ser.deserialize_message(&sm, &ros_msg);

            const Timestamp ts = rclcpp::Time(ros_msg.header.stamp).nanoseconds();
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(ros_msg, cloud);

            if (msg->topic_name == target_topic_) {
                if (in_window_count > 0) tgt_scans.push_back({ts, std::move(cloud)});
                continue;
            }

            const bool in_window = (src_seen % period) < BATCH;
            ++src_seen;
            if (!in_window) continue;

            if (in_window_count == 0) batch_start_idx = src_seen;
            src_scans.push_back({ts, std::move(cloud)});
            if (++in_window_count == BATCH) {
                auto cal_clean =
                    buildCalibrator(src_scans, tgt_scans,
                                    [](const pcl::PointCloud<pcl::PointXYZ>& c) { return c; });
                const auto r_clean = cal_clean->solve();
                if (r_clean.extrinsics.empty()) {
                    RCLCPP_WARN(get_logger(), "Batch %ld: clean solve failed, skipping",
                                batch_start_idx);
                } else if (!(r_clean.twist_converged && r_clean.extrinsic_converged)) {
                    RCLCPP_WARN(get_logger(), "Batch %ld: clean solve did not converge, skipping",
                                batch_start_idx);
                } else {
                    const gtsam::Pose3 T_clean = r_clean.extrinsics.at(target_topic_);

                    std::vector<gtsam::Pose3> perturbed = runBootstrap(src_scans, tgt_scans);
                    if (static_cast<int>(perturbed.size()) < 3) {
                        RCLCPP_WARN(get_logger(),
                                    "Batch %ld: only %zu bootstrap runs converged, skipping",
                                    batch_start_idx, perturbed.size());
                    } else {
                        const BatchEstimate est = makeBatchEstimate(T_clean, perturbed);
                        batch_estimates.push_back(est);
                        logBatch(r_clean, est, batch_start_idx, N, csv);

                        const FusionResult fr =
                            fuseBatchEstimates(batch_estimates, extrinsic_init_);
                        const gtsam::Pose3& Tf_n = fr.T_fused;

                        const auto& C = fr.cov_independent;
                        rec << batch_estimates.size() << ","
                            << Tf_n.rotation().roll() * 180.0 / M_PI << ","
                            << Tf_n.rotation().pitch() * 180.0 / M_PI << ","
                            << Tf_n.rotation().yaw() * 180.0 / M_PI << "," << Tf_n.translation().x()
                            << "," << Tf_n.translation().y() << "," << Tf_n.translation().z();
                        for (int i = 0; i < 6; ++i)
                            for (int j = i; j < 6; ++j) rec << "," << C(i, j);
                        rec << "," << batch_estimates.size() << "\n";
                        rec.flush();
                    }
                }

                ++batch_idx;
                in_window_count = 0;
                src_scans.clear();
                tgt_scans.clear();
            }
        }

        RCLCPP_INFO(get_logger(), "Done → %s/calibration.csv", output_dir_.c_str());

        if (batch_estimates.empty()) {
            RCLCPP_ERROR(get_logger(), "No valid batch estimates");
            return;
        }

        const FusionResult fused = fuseBatchEstimates(batch_estimates, extrinsic_init_);
        const gtsam::Pose3& Tf = fused.T_fused;
        const auto& C_ind = fused.cov_independent;
        const auto& C_sand = fused.cov_sandwich;

        std::ofstream yaml(output_dir_ + "/fused_result.yaml");
        yaml << std::fixed << std::setprecision(9);
        yaml << "n_batches: " << batch_estimates.size() << "\n";
        yaml << "translation:\n"
             << "  x: " << Tf.translation().x() << "\n"
             << "  y: " << Tf.translation().y() << "\n"
             << "  z: " << Tf.translation().z() << "\n";
        yaml << "rotation_rpy_deg:\n"
             << "  roll:  " << Tf.rotation().roll() * 180.0 / M_PI << "\n"
             << "  pitch: " << Tf.rotation().pitch() * 180.0 / M_PI << "\n"
             << "  yaw:   " << Tf.rotation().yaw() * 180.0 / M_PI << "\n";
        yaml << "se3_row_major: [";
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) yaml << Tf.matrix()(r, c) << (r == 3 && c == 3 ? "" : ", ");
        yaml << "]\n";

        auto write3sig = [&](const Eigen::Matrix<double, 6, 6>& C, const std::string& key) {
            Eigen::Matrix<double, 6, 1> s = 3.0 * C.diagonal().cwiseSqrt();
            yaml << key << ":\n"
                 << "  rot_deg: [" << s(0) * 180 / M_PI << ", " << s(1) * 180 / M_PI << ", "
                 << s(2) * 180 / M_PI << "]\n"
                 << "  trans_m: [" << s(3) << ", " << s(4) << ", " << s(5) << "]\n";
        };
        write3sig(C_ind, "three_sigma_independent");
        write3sig(C_sand, "three_sigma_sandwich");

        RCLCPP_INFO(get_logger(),
                    "Fused (%zu batches): T t=(%.4f, %.4f, %.4f)  r=(%.3f, %.3f, %.3f) deg",
                    batch_estimates.size(), Tf.translation().x(), Tf.translation().y(),
                    Tf.translation().z(), Tf.rotation().roll() * 180.0 / M_PI,
                    Tf.rotation().pitch() * 180.0 / M_PI, Tf.rotation().yaw() * 180.0 / M_PI);
        RCLCPP_INFO(get_logger(), "Fused result → %s/fused_result.yaml", output_dir_.c_str());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarCalibrationNode>();
    rclcpp::shutdown();
    return 0;
}

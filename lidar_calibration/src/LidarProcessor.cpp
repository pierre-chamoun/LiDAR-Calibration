#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>

#include <Eigen/SVD>
#include <lidar_calibration/LidarProcessor.hpp>
#include <stdexcept>

namespace {

Eigen::Matrix3d normalizeCovariance(const Eigen::Matrix3d& C) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(C, Eigen::ComputeFullU);
    const double s = svd.singularValues()(0);
    Eigen::Vector3d sv;
    sv << s, s, s * 1e-3;
    return svd.matrixU() * sv.asDiagonal() * svd.matrixU().transpose();
}

std::vector<Eigen::Matrix3d> computeCovariances(pcl::search::KdTree<pcl::PointXYZ>::Ptr tree,
                                                int k_neighbors) {
    const auto& cloud = *(tree->getInputCloud());
    std::vector<Eigen::Matrix3d> covs(cloud.size());
    std::vector<int> nn_idx;
    std::vector<float> nn_dist;
    nn_idx.reserve(k_neighbors);
    nn_dist.reserve(k_neighbors);

    for (std::size_t i = 0; i < cloud.size(); ++i) {
        tree->nearestKSearch(cloud[i], k_neighbors, nn_idx, nn_dist);

        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (int j : nn_idx) mean += cloud[j].getVector3fMap().cast<double>();
        mean /= static_cast<double>(nn_idx.size());

        Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
        for (int j : nn_idx) {
            Eigen::Vector3d d = cloud[j].getVector3fMap().cast<double>() - mean;
            C += d * d.transpose();
        }
        C /= static_cast<double>(nn_idx.size());

        covs[i] = normalizeCovariance(C);
    }
    return covs;
}

}  // namespace

LidarProcessor::LidarProcessor(Config cfg) : cfg_(cfg) {}

const ProcessedLidarMeasurement* LidarProcessor::get(const LidarChannel& channel, Timestamp t) {
    auto& sensor_cache = cache_[channel.id];
    if (const auto* cached = sensor_cache.at(t)) return &cached->data;
    const auto* entry = channel.series.at(t);
    if (!entry) return nullptr;
    sensor_cache.insert(t, compute(entry->data));
    return &sensor_cache.at(t)->data;
}

ProcessedLidarMeasurement LidarProcessor::compute(const LidarMeasurement& cloud) const {
    pcl::PointCloud<pcl::PointXYZ> filtered;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloud, filtered, indices);

    if (cfg_.voxel_size > 0.0f) {
        pcl::PointCloud<pcl::PointXYZ> voxelized;
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(filtered.makeShared());
        vg.setLeafSize(cfg_.voxel_size, cfg_.voxel_size, cfg_.voxel_size);
        vg.filter(voxelized);
        filtered = std::move(voxelized);
    }

    auto cloud_ptr = filtered.makeShared();
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(cloud_ptr);

    return ProcessedLidarMeasurement{cloud_ptr, tree, computeCovariances(tree, cfg_.k_neighbors)};
}

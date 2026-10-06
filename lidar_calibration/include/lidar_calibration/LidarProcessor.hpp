#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Core>
#include <lidar_calibration/Sensors.hpp>
#include <unordered_map>
#include <vector>

struct ProcessedLidarMeasurement {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
    std::vector<Eigen::Matrix3d> covariances;
};

class LidarProcessor {
   public:
    struct Config {
        float voxel_size = 0.2f;
        int k_neighbors = 25;
    };

    explicit LidarProcessor(Config cfg);

    const ProcessedLidarMeasurement* get(const LidarChannel& channel, Timestamp t);

    void clearCache() { cache_.clear(); }
    void clearCacheBefore(Timestamp t) {
        for (auto& [_, s] : cache_) s.eraseBefore(t);
    }

   private:
    Config cfg_;
    std::unordered_map<SensorId, TimeSeries<ProcessedLidarMeasurement>> cache_;

    ProcessedLidarMeasurement compute(const LidarMeasurement& cloud) const;
};

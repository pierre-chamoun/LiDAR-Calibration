#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

#include <lidar_calibration/TimeSeries.hpp>
#include <string>
#include <unordered_map>
#include <vector>

using SensorId = std::string;

struct PointXYZT {
    PCL_ADD_POINT4D;
    float t;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, t, t))

using LidarMeasurement = pcl::PointCloud<pcl::PointXYZ>;

template <typename T>
struct SensorChannel {
    SensorId id;
    TimeSeries<T> series;
};

using LidarChannel = SensorChannel<LidarMeasurement>;

class MultiSensorChannel {
   public:
    void insertLidarMeasurement(const SensorId& id, Timestamp t, LidarMeasurement cloud) {
        auto& channel = lidar_channels_[id];
        channel.id = id;
        channel.series.insert(t, std::move(cloud));
    }

    const LidarChannel& lidarChannel(const SensorId& id) const { return lidar_channels_.at(id); }

    bool hasLidar() const { return !lidar_channels_.empty(); }

    void eraseBefore(Timestamp t) {
        for (auto& [_, ch] : lidar_channels_) ch.series.eraseBefore(t);
    }

    std::vector<SensorId> lidarIds() const {
        std::vector<SensorId> ids;
        ids.reserve(lidar_channels_.size());
        for (const auto& [id, _] : lidar_channels_) ids.push_back(id);
        return ids;
    }

   private:
    std::unordered_map<SensorId, LidarChannel> lidar_channels_;
};

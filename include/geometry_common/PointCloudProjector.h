#ifndef KELO_POINTCLOUD_PROJECTOR_H
#define KELO_POINTCLOUD_PROJECTOR_H

#include <iostream>
#include <string>
#include <functional>

#include <yaml-cpp/yaml.h>

#include <geometry_common/Point2D.h>
#include <geometry_common/Point3D.h>

namespace kelo
{

typedef std::function<bool (const kelo::geometry_common::Point3D& )>
    ValidityFunction;

class PointCloudProjector
{
    public:
        using Ptr = std::shared_ptr<PointCloudProjector>;
        using ConstPtr = std::shared_ptr<const PointCloudProjector>;

        PointCloudProjector();
        virtual ~PointCloudProjector();

        void configureTransform(
                float cam_x = 0.0f,
                float cam_y = 0.0f,
                float cam_z = 0.0f,
                float cam_roll = 0.0f,
                float cam_pitch = 0.0f,
                float cam_yaw = 0.0f);

        bool configure(
                float cam_x = 0.0f,
                float cam_y = 0.0f,
                float cam_z = 0.0f,
                float cam_roll = 0.0f,
                float cam_pitch = 0.0f,
                float cam_yaw = 0.0f,
                float passthrough_min_z = 0.0f,
                float passthrough_max_z = 2.0f,
                float radial_dist_min = 0.1f,
                float radial_dist_max = 1000.0f,
                float angle_min = -3.14f,
                float angle_max = 3.14f,
                float angle_increment = 0.01f);

        bool configure(
                const YAML::Node& config_params_yaml);

        bool configure(
                const std::string& config_file);

        kelo::geometry_common::PointCloud3D transformAndFilterPointCloud(
                const kelo::geometry_common::PointCloud3D& cloud_in) const;

        std::vector<float> pointCloudToScan(
                const kelo::geometry_common::PointCloud3D& cloud_in,
                float angle_min,
                float angle_max) const;

        std::vector<float> pointCloudToProjectedScan(
                const kelo::geometry_common::PointCloud3D& cloud_in,
                float angle_min,
                float angle_max) const;

        std::vector<float> pointCloudToProjectedScan(
                const kelo::geometry_common::PointCloud3D& cloud_in,
                kelo::geometry_common::PointCloud3D& filtered_cloud,
                float angle_min,
                float angle_max) const;

        std::vector<float> pointCloudToProjectedScan(
                const kelo::geometry_common::PointCloud3D& cloud_in) const;

        std::vector<float> pointCloudToProjectedScan(
                const kelo::geometry_common::PointCloud3D& cloud_in,
                kelo::geometry_common::PointCloud3D& filtered_cloud) const;

        kelo::geometry_common::PointCloud2D pointCloudToProjectedPointCloud(
                const kelo::geometry_common::PointCloud3D& cloud_in) const;

        kelo::geometry_common::PointCloud2D pointCloudToProjectedPointCloud(
                const kelo::geometry_common::PointCloud3D& cloud_in,
                kelo::geometry_common::PointCloud3D& filtered_cloud) const;

        float getRadialDistMax() const;

        float getRadialDistMin() const;

        void setValidityFunction(
                ValidityFunction vf);

        void setPassthroughMinZ(
                float passthrough_min_z);

        void setPassthroughMaxZ(
                float passthrough_max_z);

        static size_t calculateNumOfScanPts(
                float angle_min,
                float angle_max,
                float angle_increment);

    private:
        std::vector<float> camera_to_target_tf_mat_;
        float passthrough_min_z_{0.0f};
        float passthrough_max_z_{2.0f};
        float radial_dist_min_{0.1f};
        float radial_dist_max_{1e3f};
        float radial_dist_min_sq_{0.01f};
        float radial_dist_max_sq_{1e6f};
        float angle_min_{-M_PI};
        float angle_max_{M_PI};
        bool is_angle_flipped_{false};
        float angle_increment_{0.01f};
        float angle_increment_inv_{100.0f};
        size_t num_of_scan_pts_{0};

        ValidityFunction external_validity_func_{nullptr};

        bool isPointValid(
                const kelo::geometry_common::Point3D& pt) const;

};

}; // namespace kelo
#endif // KELO_POINTCLOUD_PROJECTOR_H

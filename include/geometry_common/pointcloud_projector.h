#ifndef POINTCLOUD_PROJECTOR_H
#define POINTCLOUD_PROJECTOR_H

#include <iostream>
#include <string>
#include <functional>

#include <yaml-cpp/yaml.h>

#include <geometry_common/utils.h>
#include <geometry_common/point.h>

typedef std::function<bool (const geometry_common::Point& )> ValidityFunction;

class PointCloudProjector
{
    public:
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

        geometry_common::PointCloud transformAndFilterPointCloud(
                const geometry_common::PointCloud& cloud_in) const;

        std::vector<float> pointCloudToScan(
                const geometry_common::PointCloud& cloud_in) const;

        std::vector<float> pointCloudToProjectedScan(
                const geometry_common::PointCloud& cloud_in) const;

        std::vector<float> pointCloudToProjectedScan(
                const geometry_common::PointCloud& cloud_in,
                geometry_common::PointCloud& filtered_cloud) const;

        geometry_common::PointCloud pointCloudToProjectedPointCloud(
                const geometry_common::PointCloud& cloud_in) const;

        geometry_common::PointCloud pointCloudToProjectedPointCloud(
                const geometry_common::PointCloud& cloud_in,
                geometry_common::PointCloud& filtered_cloud) const;

        float getRadialDistMax() const;

        float getRadialDistMin() const;

        void setValidityFunction(
                ValidityFunction vf);

        void setPassthroughMinZ(
                float passthrough_min_z);

        void setPassthroughMaxZ(
                float passthrough_max_z);

    private:
        std::vector<float> camera_to_target_tf_mat_;
        float passthrough_min_z_, passthrough_max_z_;
        float radial_dist_min_, radial_dist_max_;
        float radial_dist_min_sq_, radial_dist_max_sq_;
        float angle_min_, angle_max_;
        bool is_angle_flipped_;
        float angle_increment_, angle_increment_inv_;
        size_t num_of_scan_pts_;

        ValidityFunction external_validity_func_;

        bool isPointValid(
                const geometry_common::Point& pt) const;

};

#endif // POINTCLOUD_PROJECTOR_H

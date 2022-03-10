/******************************************************************************
 * Copyright (c) 2022
 * KELO Robotics GmbH
 *
 * Author:
 * Dharmin Bakaraniya
 * Sushant Chavan
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#ifndef KELO_GEOMETRY_COMMON_UTILS_H
#define KELO_GEOMETRY_COMMON_UTILS_H

#include <vector>
#include <string>

#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <geometry_common/Pose2D.h>
#include <geometry_common/Point3D.h>
#include <geometry_common/LineSegment2D.h>
#include <geometry_common/Enums.h>

namespace kelo::geometry_common
{

/**
 * @brief 
 * 
 */
class Utils
{
    public:
        /**
         * @brief 
         * 
         * @param value 
         * @param decimal_places 
         * @return float 
         */
        static float roundFloat(
                float value,
                unsigned decimal_places);

        /**
         * @brief
         * 
         * @tparam T 
         * @param points 
         * @param start_index 
         * @param end_index 
         * @return T 
         */
        template <typename T>
        static T calcMeanPoint(
                const std::vector<T>& points,
                unsigned start_index,
                unsigned end_index);

        /**
         * @brief
         * 
         * @tparam T 
         * @param points 
         * @return T 
         */
        template <typename T>
        static T calcMeanPoint(
                const std::vector<T>& points);

        /**
         * @brief
         * 
         * @tparam T 
         * @param poses 
         * @return Pose2D 
         */
        template <typename T>
        static Pose2D calcMeanPose(
                const T& poses);

        /**
         * @brief
         * 
         * @tparam T 
         * @param points 
         * @param pt 
         * @return T 
         */
        template <typename T>
        static T calcClosestPoint(
                const std::vector<T>& points,
                const T& pt = T());

        /**
         * @brief 
         * 
         * @param points 
         * @param cluster_distance_threshold 
         * @param min_cluster_size 
         * @return std::vector<PointCloud2D> 
         */
        static std::vector<PointCloud2D> clusterPoints(
                const PointCloud2D& points,
                float cluster_distance_threshold = 0.1f,
                size_t min_cluster_size = 3);

        /**
         * @brief 
         * 
         * @param points 
         * @param cluster_distance_threshold 
         * @param min_cluster_size 
         * @return std::vector<PointCloud2D> 
         */
        static std::vector<PointCloud2D> clusterOrderedPoints(
                const PointCloud2D& points,
                float cluster_distance_threshold = 0.1f,
                size_t min_cluster_size = 3);

        /**
         * @brief
         * 
         * @param vel 
         * @param num_of_poses 
         * @param future_time 
         * @return std::vector<Pose2D> 
         */
        static std::vector<Pose2D> calcTrajectory(
                const Velocity2D& vel,
                size_t num_of_poses,
                float future_time);

        /**
         * @brief
         * 
         * @param angle1 
         * @param angle2 
         * @return float 
         */
        static float calcShortestAngle(
                float angle1,
                float angle2);

        /**
         * @brief 
         * 
         * @param m 
         * @param c 
         * @param p 
         * @param perpendicular_m 
         * @param perpendicular_c 
         */
        static void findPerpendicularLineAt(
                float m,
                float c,
                const Point2D& p,
                float& perpendicular_m,
                float& perpendicular_c);

        /**
         * @brief 
         * 
         * @param m 
         * @param c 
         * @param p 
         * @return float 
         */
        static float calcSquaredDistToLine(
                float m,
                float c,
                const Point2D& p);

        /**
         * @brief
         * 
         * @param m 
         * @param c 
         * @param p 
         * @return Point2D 
         */
        static Point2D calcProjectedPointOnLine(
                float m,
                float c,
                const Point2D& p);

        /**
         * @brief Get the Projected Point On Line object
         * 
         * @param line_start 
         * @param line_end 
         * @param p 
         * @param is_segment 
         * @return Point2D 
         */
        static Point2D calcProjectedPointOnLine(
                const Point2D& line_start,
                const Point2D& line_end,
                const Point2D& p,
                bool is_segment);

        /**
         * @brief
         * 
         * @param m 
         * @param c 
         * @param p 
         * @return Point2D 
         */
        static Point2D calcProjectedPointOnMajorAxis(
                float m,
                float c,
                const Point2D& p);

        /**
         * @brief Find the distance of point p from a line formed by points a and b
         * source: https://stackoverflow.com/a/1501725/10460994
         * 
         * @param a 
         * @param b 
         * @param p 
         * @param is_segment 
         * @return float 
         */
        static float calcSquaredDistToLine(
                const Point2D& a,
                const Point2D& b,
                const Point2D& p,
                bool is_segment = false);

        /**
         * @brief 
         * 
         * @param pts 
         * @param start_index 
         * @param end_index 
         * @param m 
         * @param c 
         * @param delta 
         * @param itr_limit 
         * @return float 
         */
        static float fitLineRANSAC(
                const PointCloud2D& pts,
                unsigned start_index,
                unsigned end_index,
                float& m,
                float& c,
                float delta = 0.2f,
                size_t itr_limit = 10);

        /**
         * @brief 
         * 
         * @param pts 
         * @param m 
         * @param c 
         * @param delta 
         * @param itr_limit 
         * @return float 
         */
        static float fitLineRANSAC(
                const PointCloud2D& pts,
                float& m, float& c,
                float delta = 0.2f,
                size_t itr_limit = 10);

        /**
         * @brief 
         * 
         * @param pts 
         * @param start_index 
         * @param end_index 
         * @param line_segment 
         * @param delta 
         * @param itr_limit 
         * @return float 
         */
        static float fitLineSegmentRANSAC(
                const PointCloud2D& pts,
                unsigned start_index,
                unsigned end_index,
                LineSegment2D& line_segment,
                float delta = 0.2f,
                size_t itr_limit = 10);

        /**
         * @brief 
         * 
         * @param pts 
         * @param line_segment 
         * @param delta 
         * @param itr_limit 
         * @return float 
         */
        static float fitLineSegmentRANSAC(
                const PointCloud2D& pts,
                LineSegment2D& line_segment,
                float delta = 0.2f,
                size_t itr_limit = 10);

        /**
         * @brief 
         * 
         * @param pts 
         * @param score_threshold 
         * @param delta 
         * @param itr_limit 
         * @return std::vector<LineSegment2D> 
         */
        static std::vector<LineSegment2D> fitLineSegmentsRANSAC(
                const PointCloud2D& pts,
                float score_threshold = 0.9f,
                float delta = 0.2f,
                size_t itr_limit = 10);

        /**
         * @brief 
         * 
         * @param pts 
         * @param start_index 
         * @param end_index 
         * @param line_segment 
         * @return float 
         */
        static float fitLineRegression(
                const PointCloud2D& pts,
                unsigned start_index,
                unsigned end_index,
                LineSegment2D& line_segment);

        /**
         * @brief 
         * 
         * @param pts 
         * @param line_segment 
         * @return float 
         */
        static float fitLineRegression(
                const PointCloud2D& pts,
                LineSegment2D& line_segment);

        /**
         * @brief 
         * 
         * @param pts 
         * @param error_threshold 
         * @return std::vector<LineSegment2D> 
         */
        static std::vector<LineSegment2D> applyPiecewiseRegression(
                const PointCloud2D& pts,
                float error_threshold = 0.1f);

        /**
         * @brief 
         * 
         * @param pts 
         * @param error_threshold 
         * @return std::vector<LineSegment2D> 
         */
        static std::vector<LineSegment2D> applyPiecewiseRegressionSplit(
                const PointCloud2D& pts,
                float error_threshold = 0.1f);

        /**
         * @brief 
         * 
         * @param line_segments 
         * @param distance_threshold 
         * @param angle_threshold 
         */
        static void mergeCloseLines(
                std::vector<LineSegment2D>& line_segments,
                float distance_threshold = 0.2,
                float angle_threshold = 0.2);

        /**
         * @brief 
         * 
         * @param line_segments 
         * @param distance_threshold 
         * @param angle_threshold 
         */
        static void mergeCloseLinesBF(
                std::vector<LineSegment2D>& line_segments,
                float distance_threshold = 0.2,
                float angle_threshold = 0.2);

        /**
         * @brief 
         * 
         * @param pts 
         * @param regression_error_threshold 
         * @param distance_threshold 
         * @param angle_threshold 
         * @return std::vector<LineSegment2D> 
         */
        static std::vector<LineSegment2D> fitLineSegments(
                const std::vector<Point2D>& pts,
                float regression_error_threshold = 0.1f,
                float distance_threshold = 0.2,
                float angle_threshold = 0.2);

        /**
         * @brief 
         * 
         * @param value 
         * @param max_limit 
         * @param min_limit 
         * @return float 
         */
        static float clip(
                float value,
                float max_limit,
                float min_limit);

        /**
         * @brief clip `value` to be between `min_limit` and `max_limit` while
         * maintaining sign.\n
         * If `value` is positive, it clips in the range [min_limit, max_limit]
         * If `value` is negative, it clips in the range [-max_limit, -min_limit]
         *
         * @note: Both min_limit and max_limit needs to be positive values
         * 
         * @param value number to be clipped
         * @param max_limit positive maximum limit
         * @param min_limit positive minimum limit
         * @return float
         */
        static float clipSigned(
                float value,
                float max_limit,
                float min_limit);

        /**
         * @brief convert any angle value to be between -pi and pi
         * 
         * @param raw_angle an angular value in radians
         * @return float angular value in radians in range [-pi, pi]
         */
        static float clipAngle(
                float raw_angle);

        /**
         * @brief 
         * 
         * @param vel 
         * @param max_vel 
         * @param min_vel 
         * @return Velocity2D 
         */
        static Velocity2D applyVelLimits(
                const Velocity2D& vel,
                const Velocity2D& max_vel,
                const Velocity2D& min_vel);

        /**
         * @brief 
         * 
         * @param cmd_vel 
         * @param curr_vel 
         * @param max_acc 
         * @param loop_rate 
         * @return Velocity2D 
         */
        static Velocity2D applyAccLimits(
                const Velocity2D& cmd_vel,
                const Velocity2D& curr_vel,
                const Acceleration2D& max_acc,
                float loop_rate);

        /**
         * @brief 
         * 
         * @param src 
         * @param target 
         * @param t 
         * @return float 
         */
        static float applyLinearInterpolation(
                float src,
                float target,
                float t);

        /**
         * @brief Convert from PointCloud2D or PointCloud3D to
         * sensor_msgs::PointCloud
         * 
         * @tparam T Point2D or Point3D
         * @param pc PointCloud2D or PointCloud3D
         * @param frame 
         * @return sensor_msgs::PointCloud 
         */
        template <typename T>
        sensor_msgs::PointCloud convertToROSPointCloud(
                const std::vector<T>& pc,
                const std::string& frame = "base_link");

        /**
         * @brief 
         * 
         * @param pc 
         * @return PointCloud3D 
         */
        static PointCloud3D convertToPointCloud3D(
                const sensor_msgs::PointCloud& pc);

        /**
         * @brief `row_sub_sample_factor` and `col_sub_sample_factor` parameterise how
         * many points are skipped. \n
         * e.g. if both their values is 1, no points are skipped \n
         *      if both their values are 2 and it is an organised cloud, the
         *      resulting cloud will be 1/4th the input cloud
         * 
         * @param cloud_msg 
         * @param row_sub_sample_factor 
         * @param col_sub_sample_factor 
         * @return PointCloud3D 
         */
        static PointCloud3D convertToPointCloud3D(
                const sensor_msgs::PointCloud2& cloud_msg,
                size_t row_sub_sample_factor = 1,
                size_t col_sub_sample_factor = 1);

        /**
         * @brief Convert from LaserScan msg to PointCloud
         *
         * @tparam T Point2D or Point3D
         * @param scan laser scan message
         *
         * @return PointCloud2D or PointCloud3D
         */
        template <typename T>
        static std::vector<T> convertToPointCloud(
                const sensor_msgs::LaserScan& scan);

        /**
         * @brief
         * 
         * @param angle 
         * @return float 
         */
        static float calcPerpendicularAngle(
                float angle);

        /**
         * @brief
         * 
         * @param angle 
         * @return float 
         */
        static float calcReverseAngle(
                float angle);

        /**
         * @brief 
         * 
         * @note Since the angles wrap around, the rule of thumb here is that the sector
         * considered is the one where you move counter-clockwise from min_angle and
         * all angles are always represented from -pi to pi
         * 
         * @param angle 
         * @param max_angle 
         * @param min_angle 
         * @return bool 
         */
        static bool isAngleWithinBounds(
                float angle,
                float max_angle,
                float min_angle);

        /**
         * @brief 
         * 
         * @param start 
         * @param end 
         * @param pt 
         * @param max_perp_dist 
         * @param step_size 
         * @return std::vector<Point2D> 
         */
        static std::vector<Point2D> generatePerpendicularPointsAt(
                const Pose2D& pose,
                float max_perp_dist = 3.0f,
                float step_size = 0.1f);

        /**
         * @brief
         * 
         * @param a 
         * @param b 
         * @param c 
         * @return float 
         */
        static float calcAngleBetweenPoints(
                const Point2D& a,
                const Point2D& b,
                const Point2D& c);

        /**
         * @brief 
         * 
         * @param a 
         * @param b 
         * @param c 
         * @return WindingOrder 
         */
        static WindingOrder calcWindingOrder(
                const Point2D& a,
                const Point2D& b,
                const Point2D& c);

        /**
         * @brief Convert from Quaternion to Euler angles
         *
         * @param qx
         * @param qy
         * @param qz
         * @param qw
         * @param roll rotation on X axis
         * @param pitch rotation on Y axis
         * @param yaw rotation on Z axis
         */
        static void convertQuaternionToEuler(
                float qx,
                float qy,
                float qz,
                float qw,
                float& roll,
                float& pitch,
                float& yaw);

        /**
         * @brief Convert from Euler to Quaternion angles
         *
         * @param roll rotation on X axis
         * @param pitch rotation on Y axis
         * @param yaw rotation on Z axis
         * @param qx
         * @param qy
         * @param qz
         * @param qw
         */
        static void convertEulerToQuaternion(
                float roll,
                float pitch,
                float yaw,
                float& qx,
                float& qy,
                float& qz,
                float& qw);

        /**
         * @brief
         * 
         * @param trajectory 
         * @param frame 
         * @return nav_msgs::Path 
         */
        static nav_msgs::Path convertToROSPath(
                const std::vector<Pose2D>& trajectory,
                const std::string& frame);

        /**
         * @brief
         * 
         * @param geometric_path 
         * @param frame 
         * @param red 
         * @param green 
         * @param blue 
         * @param alpha 
         * @param line_width 
         * @return visualization_msgs::Marker 
         */
        static visualization_msgs::Marker convertGeometricPathToMarker(
                const std::vector<Pose2D>& geometric_path,
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.05f);

        /**
         * @brief convert PointCloud2D or PointCloud3D to Marker
         *
         * @tparam T Point3D or Point3D
         * @param cloud PointCloud2D or PointCloud3D
         * @param frame
         * @param diameter
         * @param red
         * @param green
         * @param blue
         * @param alpha
         *
         * @return 
         */
        template <typename T>
        visualization_msgs::Marker convertPointCloudToMarker(
                const std::vector<T>& cloud,
                const std::string& frame = "base_link",
                float diameter = 0.05f,
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f);

        /**
         * @brief
         * 
         * @param string_label 
         * @param frame 
         * @param red 
         * @param green 
         * @param blue 
         * @param alpha 
         * @param size 
         * @return visualization_msgs::Marker 
         */
        static visualization_msgs::Marker convertStringToMarker(
                const std::string& string_label,
                const std::string& frame,
                float red = 0.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float size = 0.2f);
};

} // namespace kelo::geometry_common

#endif // KELO_GEOMETRY_COMMON_UTILS_H

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

#ifndef KELO_GEOMETRY_COMMON_POSE_2D_H
#define KELO_GEOMETRY_COMMON_POSE_2D_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

#include <cmath>

#include <geometry_common/Point2D.h>

namespace kelo::geometry_common
{

// Forward declaration 
class TransformMat2D;

/**
 * @brief 
 * 
 */
class Pose2D
{
    public:
        float x, y, theta;

        using Ptr = std::shared_ptr<Pose2D>;
        using ConstPtr = std::shared_ptr<const Pose2D>;

        /**
         * @brief
         * 
         * @param _x 
         * @param _y 
         * @param _theta 
         */
        Pose2D(float _x = 0.0f, float _y = 0.0f, float _theta = 0.0f):
            x(_x), y(_y), theta(_theta) {}

        /**
         * @brief
         * 
         * @param pose 
         */
        Pose2D(const Pose2D& pose):
            x(pose.x), y(pose.y), theta(pose.theta) {}

        /**
         * @brief
         * 
         * @param pose 
         */
        Pose2D(const geometry_msgs::PoseStamped& pose);

        /**
         * @brief
         * 
         * @param pose 
         */
        Pose2D(const geometry_msgs::Pose& pose);

        /**
         * @brief
         * 
         * @param mat 
         */
        Pose2D(const TransformMat2D& mat);

        /**
         * @brief
         * 
         * @param stamped_transform 
         */
        Pose2D(const tf::StampedTransform& stamped_transform);

        /**
         * @brief
         * 
         */
        virtual ~Pose2D() {}

        /**
         * @brief
         * 
         * @param frame 
         * @return geometry_msgs::PoseStamped 
         */
        geometry_msgs::PoseStamped getPoseStamped(const std::string& frame="map") const;

        /**
         * @brief
         * 
         * @return geometry_msgs::Pose 
         */
        geometry_msgs::Pose getPose() const;

        /**
         * @brief
         * 
         * @return TransformMat2D
         */
        TransformMat2D getMat() const;

        /**
         * @brief
         * 
         * @param frame 
         * @param red 
         * @param green 
         * @param blue 
         * @param alpha 
         * @param size_x 
         * @param size_y 
         * @param size_z 
         * @return visualization_msgs::Marker 
         */
        visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float size_x = 0.3f,
                float size_y = 0.05f,
                float size_z = 0.05f) const;

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float getCartDist(const Pose2D& p) const
        {
            return std::sqrt(getCartDistSquared(p));
        };

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float getCartDistSquared(const Pose2D& p) const
        {
            return std::pow(x - p.x, 2) + std::pow(y - p.y, 2);
        };

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float getCartDist(const Point2D& p) const
        {
            return std::sqrt(std::pow(x - p.x, 2) + std::pow(y - p.y, 2));
        };

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string str() const;

        /**
         * @brief 
         * 
         * @param p1 
         * @param p2 
         * @return Pose2D 
         */
        Pose2D operator - (const Pose2D& p) const;

        /**
         * @brief 
         * 
         * @param pose 
         * @param scalar 
         * @return Pose2D 
         */
        Pose2D operator * (float scalar) const;

        /**
         * @brief 
         * 
         * @param p1 
         * @param p2 
         * @return bool 
         */
        bool operator == (const Pose2D& p) const;

        /**
         * @brief 
         * 
         * @param out 
         * @param pose_2d 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (std::ostream& out, const Pose2D& pose_2d);

};

using Velocity2D = Pose2D;
using Acceleration2D = Pose2D;

} // namespace kelo::geometry_common

#endif // KELO_GEOMETRY_COMMON_POSE_2D_H

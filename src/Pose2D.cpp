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

#include <cmath>
#include <geometry_common/Utils.h>
#include <geometry_common/TransformMatrix2D.h>
#include <geometry_common/Pose2D.h>

namespace kelo::geometry_common
{

Pose2D::Pose2D(float _x, float _y, float _theta)
{
    x = _x;
    y = _y;
    theta = Utils::clipAngle(_theta);
}

Pose2D::Pose2D(const geometry_msgs::PoseStamped& pose)
{
    x = pose.pose.position.x;
    y = pose.pose.position.y;
    float roll, pitch;
    Utils::convertQuaternionToEuler(
            pose.pose.orientation.x, pose.pose.orientation.y,
            pose.pose.orientation.z, pose.pose.orientation.w,
            roll, pitch, theta);
}

Pose2D::Pose2D(const geometry_msgs::Pose& pose)
{
    x = pose.position.x;
    y = pose.position.y;
    float roll, pitch;
    Utils::convertQuaternionToEuler(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
            roll, pitch, theta);
}

Pose2D::Pose2D(const TransformMatrix2D& tf_mat)
{
    x = tf_mat.x();
    y = tf_mat.y();
    theta = tf_mat.theta();
}

Pose2D::Pose2D(const tf::StampedTransform& stamped_transform)
{
    TransformMatrix2D tf_mat(stamped_transform);
    x = tf_mat.x();
    y = tf_mat.y();
    theta = tf_mat.theta();
}

geometry_msgs::PoseStamped Pose2D::asPoseStamped(const std::string& frame) const
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.pose = asPose();
    return pose;
}

geometry_msgs::Pose Pose2D::asPose() const
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0f;
    pose.orientation.x = 0.0f;
    pose.orientation.y = 0.0f;
    float qx, qy, qz, qw;
    Utils::convertEulerToQuaternion(0.0f, 0.0f, theta, qx, qy, qz, qw);
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    return pose;
}

TransformMatrix2D Pose2D::asMat() const
{
    return TransformMatrix2D(*this);
}

visualization_msgs::Marker Pose2D::asMarker(const std::string& frame,
        float red, float green, float blue, float alpha,
        float size_x, float size_y, float size_z) const
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::ARROW;
    // marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = size_x;
    marker.scale.y = size_y;
    marker.scale.z = size_z;
    marker.pose = asPose();
    return marker;
}

std::string Pose2D::asString() const
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "<x: " << x
        << ", y: " << y
        << ", theta: " << theta
        << ">";
    return ss.str();
}

Pose2D Pose2D::operator - (const Pose2D& pose) const
{
    Pose2D diff;
    diff.x = x - pose.x;
    diff.y = y - pose.y;
    diff.theta = Utils::calcShortestAngle(theta, pose.theta);
    return diff;
}

bool Pose2D::operator == (const Pose2D& pose) const
{
    return ( distTo(pose) < 1e-3f &&
             Utils::calcShortestAngle(theta, pose.theta) < 1e-2f );
}

std::ostream& operator << (std::ostream& out, const Pose2D& pose)
{
    out << "<x: " << pose.x
        << ", y: " << pose.y
        << ", theta: " << pose.theta
        << ">";
    return out;
}

} // namespace kelo::geometry_common

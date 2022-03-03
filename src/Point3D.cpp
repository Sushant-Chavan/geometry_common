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

#include <geometry_common/Utils.h>
#include <geometry_common/Point3D.h>

namespace kelo::geometry_common
{

geometry_msgs::Point Point3D::getPoint() const
{
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

geometry_msgs::Point32 Point3D::getPoint32() const
{
    geometry_msgs::Point32 point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

geometry_msgs::PointStamped Point3D::getPointStamped(const std::string& frame) const
{
    geometry_msgs::PointStamped point;
    // point.header.stamp = ros::Time::now();
    point.header.frame_id = frame;
    point.point = getPoint();
    return point;
}

void Point3D::normalise()
{
    float mag = magnitude();
    if ( mag > 0 )
    {
        x /= mag;
        y /= mag;
        z /= mag;
    }
}

Point3D Point3D::getNormalised() const
{
    Point3D normalised_pt(*this);
    normalised_pt.normalise();
    return normalised_pt;
}

void Point3D::transform(const std::vector<float>& tf_mat)
{
    assert(tf_mat.size() == 16);
    std::vector<float> p_vec{x, y, z, 1.0f};
    std::vector<float> transformed_p_vec = Utils::multiplyMatrixToVector(tf_mat, p_vec);
    x = transformed_p_vec[0];
    y = transformed_p_vec[1];
    z = transformed_p_vec[2];
}

Point3D Point3D::getTransformedPoint(const std::vector<float>& tf_mat) const
{
    Point3D transformed_pt(*this);
    transformed_pt.transform(tf_mat);
    return transformed_pt;
}

visualization_msgs::Marker Point3D::getMarker(const std::string& frame,
        float red, float green, float blue, float alpha, float diameter) const
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = diameter;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0f;
    return marker;
}

Point3D operator - (const Point3D& p1, const Point3D& p2)
{
    Point3D diff;
    diff.x = p1.x - p2.x;
    diff.y = p1.y - p2.y;
    diff.z = p1.z - p2.z;
    return diff;
}

Point3D operator + (const Point3D& p1, const Point3D& p2)
{
    Point3D sum;
    sum.x = p1.x + p2.x;
    sum.y = p1.y + p2.y;
    sum.z = p1.z + p2.z;
    return sum;
}

Point3D operator * (const Point3D& p1, float scalar)
{
    Point3D scaled;
    scaled.x = p1.x * scalar;
    scaled.y = p1.y * scalar;
    scaled.z = p1.z * scalar;
    return scaled;
}

std::ostream& operator << (std::ostream& out, const Point3D& point)
{
    out << "<x: " << point.x << ", y: " << point.y << ", z: " << point.z << ">";
    return out;
}

bool operator == (const Point3D& p1, const Point3D& p2)
{
    return ( p1.getCartDist(p2) < 1e-3f );
}

} // namespace kelo::geometry_common

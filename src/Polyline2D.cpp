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

#include <geometry_common/LineSegment2D.h>
#include <geometry_common/Point3D.h>
#include <geometry_common/Polyline2D.h>

namespace kelo::geometry_common
{

bool Polyline2D::isIntersecting(const LineSegment2D& line_segment) const
{
    // Conditions also ensures that there are atleast 2 vertices
    for ( unsigned int start = 0, end = start + 1; end < vertices.size(); start = end++ )
    {
        if ( LineSegment2D(vertices[start], vertices[end]).isIntersecting(line_segment) )
        {
            return true;
        }
    }
    return false;
}

void Polyline2D::transform(const std::vector<float>& tf_mat)
{
    for ( Point2D& vertex : vertices )
    {
        vertex.transform(tf_mat);
    }
}

void Polyline2D::transform(const Pose2D& tf)
{
    for ( Point2D& vertex : vertices )
    {
        vertex.transform(tf);
    }
}

Polyline2D Polyline2D::getTransformedPolyline(const std::vector<float>& tf_mat) const
{
    Polyline2D transformed_poly(*this);
    transformed_poly.transform(tf_mat);
    return transformed_poly;
}

Polyline2D Polyline2D::getTransformedPolyline(const Pose2D& tf) const
{
    Polyline2D transformed_poly(*this);
    transformed_poly.transform(tf);
    return transformed_poly;
}

visualization_msgs::Marker Polyline2D::getMarker(const std::string& frame,
        float red, float green, float blue, float alpha, float line_width,
        float z) const
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    // marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = line_width;
    marker.pose.orientation.w = 1.0f;
    if (!vertices.empty())
    {
        marker.points.reserve(vertices.size());
        for ( const Point2D& vertex : vertices )
        {
            marker.points.push_back(Point3D(vertex, z).getPoint());
        }
    }
    return marker;
}

Point2D& Polyline2D::operator [] (unsigned int index)
{
    return vertices[index];
}

const Point2D& Polyline2D::operator [] (unsigned int index) const
{
    return vertices[index];
}

std::ostream& operator<<(std::ostream& out, const Polyline2D& polyline)
{
    out << "Polyline vertices:" << std::endl;
    for (const auto& vertex : polyline.vertices)
    {
        out << vertex;
    }
    return out;
}

} // namespace kelo::geometry_common

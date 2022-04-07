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

namespace kelo
{
namespace geometry_common
{

float Polyline2D::length() const
{
    float len = 0.0f;
    for ( unsigned int start = 0, end = start + 1; end < vertices.size(); start = end++ )
    {
        len += vertices[start].distTo(vertices[end]);
    }
    return len;
}

bool Polyline2D::intersects(const LineSegment2D& line_segment) const
{
    // Conditions also ensures that there are atleast 2 vertices
    for ( size_t start = 0, end = start + 1; end < vertices.size(); start = end++ )
    {
        if ( LineSegment2D(vertices[start], vertices[end]).intersects(line_segment) )
        {
            return true;
        }
    }
    return false;
}

bool Polyline2D::calcClosestIntersectionPointWith(
        const LineSegment2D& line_segment,
        Point2D& intersection_pt) const
{
    bool intersects = false;
    double minDist = std::numeric_limits<double>::max();
    for ( unsigned int start = 0, end = start + 1; end < vertices.size(); start = end++ )
    {
        Point2D pt;
        if ( line_segment.calcIntersectionPointWith(
                    LineSegment2D(vertices[start], vertices[end]), pt) )
        {
            double dist = line_segment.start.distTo(pt);
            if (dist < minDist)
            {
                minDist = dist;
                intersection_pt = pt;
                intersects = true;
            }
        }
    }
    return intersects;
}

std::vector<LineSegment2D> Polyline2D::split(float max_segment_length) const
{
    std::vector<LineSegment2D> segments;
    for ( unsigned int start = 0, end = start + 1; end < vertices.size();
         start = end++ )
    {
        LineSegment2D edge(vertices[start], vertices[end]);
        Point2D unit_vector = edge.unitVector();
        if ( max_segment_length > 0 )
        {
            while ( edge.length() > max_segment_length )
            {
                Point2D split_point = edge.start + (unit_vector * max_segment_length);
                segments.push_back(LineSegment2D(edge.start, split_point));
                edge.start = split_point;
            }
        }
        segments.push_back(edge);
    }
    return segments;
}

visualization_msgs::Marker Polyline2D::asMarker(const std::string& frame,
        float red, float green, float blue, float alpha, float line_width) const
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
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
            marker.points.push_back(vertex.asPoint());
        }
    }
    return marker;
}

Polyline2D& Polyline2D::operator = (const Polyline2D& other)
{
    vertices = other.vertices;
    return *this;
}

Point2D& Polyline2D::operator [] (unsigned int index)
{
    return vertices[index];
}

const Point2D& Polyline2D::operator [] (unsigned int index) const
{
    return vertices[index];
}

bool Polyline2D::operator == (const Polyline2D& other) const
{
    if ( vertices.size() != other.size() )
    {
        return false;
    }

    for ( size_t i = 0; i < vertices.size(); i++ )
    {
        if ( !(vertices[i] == other[i]) )
        {
            return false;
        }
    }
    return true;
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

} // namespace geometry_common
} // namespace kelo

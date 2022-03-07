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
#include <geometry_common/Point3D.h>
#include <geometry_common/LineSegment2D.h>

namespace kelo::geometry_common
{

LineSegment2D::~LineSegment2D()
{
}

float LineSegment2D::getAngle() const
{
    Vec2D diff = end - start;
    return std::atan2(diff.y, diff.x);
}

float LineSegment2D::getLength() const
{
    return start.getCartDist(end);
}

float LineSegment2D::getSlope() const
{
    Vec2D diff = end - start;
    if ( fabs(diff.x) < 1e-6f )
    {
        diff.x = 1e-6f;
    }
    return diff.y/diff.x;
}

float LineSegment2D::getConstant() const
{
    float m = getSlope();
    return start.y - (m * start.x);
}

Point2D LineSegment2D::getCenter() const
{
    return (start + end) * 0.5f;
}

Point2D LineSegment2D::getUnitVector() const
{
    return (end - start) * (1.0f/getLength());
}

bool LineSegment2D::isIntersecting(const LineSegment2D& line_segment) const
{
    Point2D intersection_pt;
    return getIntersectionPoint(line_segment, intersection_pt);
}

bool LineSegment2D::getIntersectionPoint(
        const LineSegment2D& line_segment,
        Point2D& intersection_point) const
{
    /**
     * source: https://www.codeproject.com/Tips/862988/Find-the-Intersection-Point-of-Two-Line-Segments
     */
    Vec2D vec1 = end - start;
    Vec2D vec2 = line_segment.end - line_segment.start;
    Vec2D vec3 = line_segment.start - start;
    const float vec1_cross_vec2 = vec1.scalarCrossProduct(vec2);
	const float vec3_cross_vec1 = vec3.scalarCrossProduct(vec1);
    const float vec3_cross_vec2 = vec3.scalarCrossProduct(vec2);

	if ( ( std::abs(vec1_cross_vec2) < 1e-10f &&
           std::abs(vec3_cross_vec1) < 1e-10f ) || // the two lines are collinear
		 ( std::abs(vec1_cross_vec2) < 1e-10f &&
           std::abs(vec3_cross_vec1) > 1e-10f ) ) // the two lines are parallel and non-intersecting
    {
		return false;
	}

	const float t = vec3_cross_vec2/vec1_cross_vec2;
	const float u = vec3_cross_vec1/vec1_cross_vec2;

	if ( std::abs(vec1_cross_vec2) > 1e-10f &&
         0.0f <= t && t <= 1.0f &&
         0.0f <= u && u <= 1.0f )
	{
		intersection_point = start + (vec1 * t);
		return true;
	}

	return false; // The two line segments are not parallel but do not intersect.
}

Point2D LineSegment2D::getClosestPointFrom(const Point2D& point) const
{
    return Utils::getProjectedPointOnLine(start, end, point, true);
}

float LineSegment2D::getMinDistFrom(const Point2D& point) const
{
    return point.getCartDist(getClosestPointFrom(point));
}

bool LineSegment2D::containsPoint(
        const Point2D& point,
        float dist_threshold) const
{
    return ( getMinDistFrom(point) < dist_threshold );
}

visualization_msgs::Marker LineSegment2D::getMarker(const std::string& frame,
        float red, float green, float blue, float alpha, float line_width) const
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    // marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = line_width;
    marker.pose.orientation.w = 1.0f;
    marker.points.push_back(Point3D(start).getPoint());
    marker.points.push_back(Point3D(end).getPoint());
    return marker;
}

std::ostream& operator << (std::ostream& out, const LineSegment2D& line_segment)
{
    out << "start: " << line_segment.start << std::endl;
    out << "end: " << line_segment.end << std::endl;
    return out;
}

} // namespace kelo::geometry_common

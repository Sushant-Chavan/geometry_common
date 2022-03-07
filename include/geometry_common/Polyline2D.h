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

#ifndef KELO_GEOMETRY_COMMON_POLYLINE_2D_H
#define KELO_GEOMETRY_COMMON_POLYLINE_2D_H

#include <visualization_msgs/Marker.h>

#include <geometry_common/Point2D.h>

namespace kelo::geometry_common
{

// Forward declaration
class LineSegment2D;

/**
 * @brief Represents a polyline (or a linestring) as a sequence of consecutive 
 * 2D waypoints(or vertices).
 * 
 */
class Polyline2D
{
    public:
        PointVec2D vertices;

        using Ptr = std::shared_ptr<Polyline2D>;
        using ConstPtr = std::shared_ptr<const Polyline2D>;

        /**
         * @brief Construct a new empty Polyline2D object
         * 
         */
        Polyline2D() = default;

        /**
         * @brief Copy constructor
         * 
         * @param polyline The polyline to be copied
         */
        Polyline2D(const Polyline2D& polyline):
            vertices(polyline.vertices) {}

        /**
         * @brief Construct a new Polyline2D object from a vector of 2D points
         * 
         * @param verts An ordered vector of 2D points representing the polyline
         */
        Polyline2D(const PointVec2D& verts):
            vertices(verts) {}

        /**
         * @brief Destroy the Polyline2D object
         * 
         */
        virtual ~Polyline2D() {}

        /**
         * @brief Checks if the 2D polyline and a 2D line segment intersect at
         * atleast one point
         * 
         * @param line_segment The 2D line segment to be checked for intersection
         * @return bool True if the polyline intersects the 2D line segment at
         * atleast one point, false otherwise
         */
        virtual bool isIntersecting(const LineSegment2D& line_segment) const;

        /**
         * @brief If the 2D polyline intersects with a 2D line segment, this function returns the 
         * intersection point that is closest to the start vertex of the line segment
         * @param line_segment The 2D line segment to be checked for intersection
         * @param intersection_pt The intersection point closest to the start vertex of the line segment
         * @return bool True if the polyline intersects the 2D line segment at
         * atleast one point, false otherwise
         */
        virtual bool getClosestIntersectionPoint(
                const LineSegment2D& line_segment,
                Point2D& intersection_pt);

        /**
         * @brief Get an RViz visualization marker for the polyline object
         * 
         * @param frame The frame in which the polyline marker points are specified
         * @param red The red color-component to be used in the line marker
         * color in the range [0.0, 1.0]
         * @param green The green color-component to be used in the line marker
         * color in the range [0.0, 1.0]
         * @param blue The blue color-component to be used in the line marker
         * color in the range [0.0, 1.0]
         * @param alpha The transparency of the generated line marker
         * in the range [0.0, 1.0]
         * @param line_width The width of the line marker
         * @param z The Z-coordinate to be appended to all vertices of the line
         * string to represent the 2D point in a 3D space. (Default: 0.0)
         * @return visualization_msgs::Marker A marker object representing the polyline
         */
        virtual visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.1f,
                float z = 0.0f) const;

        /**
         * @brief Get the number of vertices representing the polyline
         * 
         * @return size_t Num of vertices
         */
        size_t size() const
        {
            return vertices.size();
        }

        /**
         * @brief Indexing operator to access each individual vertex by its index.
         * This method can be used to update the vertex data.
         * 
         * @param index The index of the vertex to be accessed.
         * @return Point2D& The polyline vertex at the specified index
         */
        Point2D& operator [] (unsigned int index);

        /**
         * @brief Indexing operator to access each individual vertex by its index.
         * The returned vertex data cannot be modified
         * 
         * @param index The index of the vertex to be accessed.
         * @return const Point2D& The polyline vertex at the specified index
         */
        const Point2D& operator [] (unsigned int index) const;

        /**
         * @brief Append the Polyline information as string to the input stream object
         * 
         * @param out The stream object to which the polyline information should be appended
         * @param polyline The polyline whose data should be appended to the stream object
         * @return std::ostream& The stream object representing the concatenation
         * of the input stream and the polyline information
         */
        friend std::ostream& operator << (std::ostream &out, const Polyline2D& polyline);
};

} // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_POLYLINE_2D_H

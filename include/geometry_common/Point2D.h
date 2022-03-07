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

#ifndef KELO_GEOMETRY_COMMON_POINT_2D_H
#define KELO_GEOMETRY_COMMON_POINT_2D_H

#include <visualization_msgs/Marker.h>

#include <cmath>

namespace kelo::geometry_common
{

// Forward declaration 
class Pose2D;

/**
 * @brief 
 * 
 */
class Point2D
{
    public:
        float x{0.0f}, y{0.0f};

        using Ptr = std::shared_ptr<Point2D>;
        using ConstPtr = std::shared_ptr<const Point2D>;

        /**
         * @brief
         * 
         * @param _x 
         * @param _y 
         */
        Point2D(float _x = 0.0f, float _y = 0.0f):
            x(_x),
            y(_y) {}

        /**
         * @brief
         * 
         * @param point 
         */
        Point2D(const Point2D& point):
            Point2D(point.x, point.y) {}

        /**
         * @brief
         * 
         */
        virtual ~Point2D() {}

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float getCartDist(const Point2D& p) const
        {
            return std::sqrt(getCartDistSquared(p));
        };

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float getCartDistSquared(const Point2D& p) const
        {
            return std::pow(x - p.x, 2) + std::pow(y - p.y, 2);
        };

        /**
         * @brief 
         * 
         * @return float 
         */
        inline float magnitude() const
        {
            return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        };

        /**
         * @brief Normalise vector in place
         * 
         */
        void normalise();

        /**
         * @brief: Return a normalised vector
         *
         * @return: Point2D
         */
        Point2D getNormalised() const;

        /**
         * @brief 
         * 
         * @param point 
         * @return float 
         */
        float scalarCrossProduct(const Point2D& point) const;

        /**
         * @brief
         * 
         * @param frame 
         * @param red 
         * @param green 
         * @param blue 
         * @param alpha 
         * @param diameter 
         * @param z 
         * @return visualization_msgs::Marker 
         */
        visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float diameter = 0.2f,
                float z = 0.0f) const;

        /**
         * @brief 
         * 
         * @param p1 
         * @param p2 
         * @return Point2D 
         */
        friend Point2D operator - (const Point2D& p1, const Point2D& p2);

        /**
         * @brief 
         * 
         * @param p1 
         * @param p2 
         * @return Point2D 
         */
        friend Point2D operator + (const Point2D& p1, const Point2D& p2);

        /**
         * @brief 
         * 
         * @param p1 
         * @param scalar 
         * @return Point2D 
         */
        friend Point2D operator * (const Point2D& p1, float scalar);

        /**
         * @brief 
         * 
         * @param out 
         * @param point 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (std::ostream& out, const Point2D& point);

        /**
         * @brief 
         * 
         * @param p1 
         * @param p2 
         * @return bool 
         */
        friend bool operator == (const Point2D& p1, const Point2D& p2);
};

/**
 * @brief 
 * 
 */
using Vec2D = Point2D;

/**
 * @brief 
 * 
 */
using PointVec2D = std::vector<Point2D>;

/**
 * @brief 
 * 
 */
using PointCloud2D = std::vector<Point2D>;

}; // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_POINT_H

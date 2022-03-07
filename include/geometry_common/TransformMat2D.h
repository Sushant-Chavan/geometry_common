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

#ifndef KELO_GEOMETRY_COMMON_TRANSFORM_MAT_2D_H
#define KELO_GEOMETRY_COMMON_TRANSFORM_MAT_2D_H

#include <array>

#include <tf/transform_datatypes.h>

namespace kelo::geometry_common
{

// Forward declaration 
class Pose2D;
class Point2D;
class Polyline2D;
class Polygon2D;
using Vec2D = Point2D;

/**
 * @brief 
 * 
 */
class TransformMat2D
{
    public:

        using Ptr = std::shared_ptr<TransformMat2D>;
        using ConstPtr = std::shared_ptr<const TransformMat2D>;

        TransformMat2D():
            TransformMat2D(0.0f, 0.0f, 0.0f) {}

        /**
         * @brief Construct transformation matrix with euler angle values
         *
         * @param x Translation in X axis
         * @param y Translation in Y axis
         * @param theta Rotation on Z axis
         */
        TransformMat2D(float x, float y, float theta);

        /**
         * @brief Construct transformation matrix with quaternion angle values
         *
         * @param x
         * @param y
         * @param qx
         * @param qy
         * @param qz
         * @param qw
         */
        TransformMat2D(float x, float y, float qx, float qy, float qz, float qw);

        TransformMat2D(const tf::StampedTransform& stamped_transform);

        TransformMat2D(const Pose2D& pose);

        TransformMat2D(const TransformMat2D& tf_mat);

        /**
         * @brief
         * 
         */
        virtual ~TransformMat2D() {}

        void update(float x, float y, float theta);

        void update(float x, float y, float qx, float qy, float qz, float qw);

        void update(const Pose2D& pose);

        void update(const TransformMat2D& tf_mat);

        TransformMat2D getInverse() const;

        void invert();

        float getX() const;

        float getY() const;

        float getTheta() const;

        std::array<float, 4> getQuaternion() const;

        std::array<float, 4> getRotationMat() const;

        Vec2D getTranslationVec() const;

        Pose2D getPose2D() const;

        void setX(float x);

        void setY(float y);

        void setTheta(float theta);

        void setQuaternion(float qx, float qy, float qz, float qw);

        void transform(Point2D& point) const;

        void transform(Pose2D& pose) const;

        void transform(Polyline2D& polyline) const;

        void transform(Polygon2D& polygon) const;

        TransformMat2D operator * (const TransformMat2D& tf_mat) const;

        TransformMat2D& operator *= (const TransformMat2D& tf_mat);

        Point2D operator * (const Point2D& vec) const;

        Pose2D operator * (const Pose2D& vec) const;

        Polyline2D operator * (const Polyline2D& polyline) const;

        Polygon2D operator * (const Polygon2D& polygon) const;

        const float& operator [] (unsigned int index) const;

        /**
         * @brief 
         * 
         * @param out 
         * @param tf_mat 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (
                std::ostream& out,
                const TransformMat2D& tf_mat);

    protected:
        std::array<float, 6> mat_;

};

}; // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_TRANSFORM_MAT_2D_H

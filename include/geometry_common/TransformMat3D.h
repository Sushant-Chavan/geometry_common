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

#include <geometry_common/Point3D.h>

namespace kelo::geometry_common
{

/**
 * @brief 
 * 
 */
class TransformMat3D
{
    public:

        using Ptr = std::shared_ptr<TransformMat3D>;
        using ConstPtr = std::shared_ptr<const TransformMat3D>;

        TransformMat3D():
            TransformMat3D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f) {}

        TransformMat3D(float x, float y, float z,
                       float roll, float pitch, float yaw);

        TransformMat3D(float x, float y, float z,
                       float qx, float qy, float qz, float qw);

        TransformMat3D(const tf::StampedTransform& stamped_transform);

        TransformMat3D(const TransformMat3D& tf_mat);

        virtual ~TransformMat3D() {};

        void update(float x, float y, float z,
                    float roll, float pitch, float yaw);

        void update(float x, float y, float z,
                    float qx, float qy, float qz, float qw);

        void update(const TransformMat3D& tf_mat);

        TransformMat3D getInverse() const;

        void invert();

        float getX() const;

        float getY() const;

        float getZ() const;

        float getRoll() const;

        float getPitch() const;

        float getYaw() const;

        std::array<float, 4> getQuaternion() const;

        std::array<float, 9> getRotationMat() const;

        Vec3D getTranslationVec() const;

        void setX(float x);

        void setY(float y);

        void setZ(float z);

        void setRoll(float roll);

        void setPitch(float pitch);

        void setYaw(float yaw);

        void setRollPitchYaw(float roll, float pitch, float yaw);

        void setQuaternion(float qx, float qy, float qz, float qw);

        void transform(Vec3D& vec) const;

        TransformMat3D operator * (const TransformMat3D& tf_mat) const;

        TransformMat3D& operator *= (const TransformMat3D& tf_mat);

        Vec3D operator * (const Vec3D& vec) const;

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
                const TransformMat3D& tf_mat);

    protected:
        std::array<float, 12> mat_;

};

}; // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_TRANSFORM_MAT_2D_H

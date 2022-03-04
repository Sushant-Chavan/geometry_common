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

#include <geometry_common/TransformMat2D.h>

namespace kelo::geometry_common
{

TransformMat2D::TransformMat2D(float x, float y, float theta)
{
    update(x, y, theta);
}

TransformMat2D::TransformMat2D(float x, float y, float qx, float qy, float qz, float qw)
{
    update(x, y, qx, qy, qz, qw);
}

TransformMat2D::TransformMat2D(const tf::StampedTransform& stamped_transform)
{
}

TransformMat2D::TransformMat2D(const Pose2D& pose)
{
    update(pose);
}

TransformMat2D::TransformMat2D(const TransformMat2D& tf_mat)
{
    update(tf_mat);
}

TransformMat2D::void update(float x, float y, float theta)
{
    mat[2] = x;
    mat[5] = y;
    setTheta(theta);
}

void TransformMat2D::update(float x, float y, float qx, float qy, float qz, float qw)
{
    float roll, pitch, yaw;
    Utils::getEulerFromQuaternion(qx, qy, qz, qw, roll, pitch, yaw);
    update(x, y, yaw);
}

void TransformMat2D::update(const Pose2D& pose)
{
    update(pose.x, pose.y, pose.theta);
}

void TransformMat2D::update(const TransformMat2D& tf_mat)
{
    for ( size_t i = 0; i < mat.size(); i++ )
    {
        mat[i] = tf_mat[i];
    }
}

TransformMat2D TransformMat2D::getInverse() const
{
    TransformMat2D inv_tf_mat(mat[2], mat[5], -getTheta());
    Vec2D transformed_vec = inv_tf_mat * Vec2D(-mat[2], -mat[5]);
    inv_tf_mat.setX(transformed_vec.x);
    inv_tf_mat.setY(transformed_vec.y);
    return inv_tf_mat;
}

void TransformMat2D::invert()
{
    TransformMat2D inv_tf_mat = getInverse();
    update(inv_tf_mat);
}

float TransformMat2D::getX() const
{
    return mat[2];
}

float TransformMat2D::getY() const
{
    return mat[5];
}

float TransformMat2D::getTheta() const
{
    return std::atan2(mat[3], mat[0]);
}

std::array<float, 4> TransformMat2D::getQuaternion() const
{
    std::array<float, 4> q;
    Utils::getQuaternionFromEuler(0.0f, 0.0f, getTheta(), q[0], q[1], q[2], q[3]);
    return q;
}

std::array<float, 4> TransformMat2D::getRotationMat() const
{
    std::array<float, 4> rot_mat;
    rot_mat[0] = mat[0];
    rot_mat[1] = mat[1];
    rot_mat[2] = mat[3];
    rot_mat[3] = mat[4];
    return rot_mat;
}

Vec2D TransformMat2D::getTranslationVec() const
{
    return Vec2D(mat[2], mat[5]);
}

Pose2D TransformMat2D::getPose2D() const
{
    return Pose2D(mat[2], mat[5], getTheta());
}

void TransformMat2D::setX(float x)
{
    mat[2] = x;
}

void TransformMat2D::setY(float y)
{
    mat[5] = x;
}

void TransformMat2D::setTheta(float theta)
{
    mat[0] = std::cos(theta);
    mat[1] = -std::sin(theta);
    mat[3] = std::sin(theta);
    mat[4] = std::cos(theta);
}

void TransformMat2D::setQuaternion(float qx, float qy, float qz, float qw)
{
    float roll, pitch, yaw;
    Utils::getEulerFromQuaternion(qx, qy, qz, qw, roll, pitch, yaw);
    setTheta(yaw);
}

TransformMat2D TransformMat2D::operator * (const TransformMat2D& tf_mat)
{
    TransformMat2D result_tf_mat(*this);
    result_tf_mat *= tf_mat;
    return result_tf_mat;
}

TransformMat2D& TransformMat2D::operator *= (const TransformMat2D& tf_mat)
{
    float arr[6];
    arr[0] = (mat[0] * tf_mat[0]) + (mat[1] * tf_mat[3]);
    arr[1] = (mat[0] * tf_mat[1]) + (mat[1] * tf_mat[4]);
    arr[2] = (mat[0] * tf_mat[2]) + (mat[1] * tf_mat[5]) + mat[2];
    arr[3] = (mat[3] * tf_mat[0]) + (mat[4] * tf_mat[3]);
    arr[4] = (mat[3] * tf_mat[1]) + (mat[4] * tf_mat[4]);
    arr[5] = (mat[3] * tf_mat[2]) + (mat[4] * tf_mat[5]) + mat[5];
    mat[0] = arr[0];
    mat[1] = arr[1];
    mat[2] = arr[2];
    mat[3] = arr[3];
    mat[4] = arr[4];
    mat[5] = arr[5];
    return *this;
}

Vec2D TransformMat2D::operator * (const Vec2D& vec)
{
    Vec2D result_vec;
    result_vec.x = (mat[0] * vec.x) + (mat[1] * vec.y) + mat[2];
    result_vec.y = (mat[3] * vec.x) + (mat[4] * vec.y) + mat[5];
    return result_vec;
}

std::ostream& operator << (std::ostream& out, const TransformMat2D& tf_mat)
{
    out << std::setprecision(3) << std::fixed;
    out << tf_mat.mat[0] << "\t"
        << tf_mat.mat[1] << "\t"
        << tf_mat.mat[2] << std::endl
        << tf_mat.mat[3] << "\t"
        << tf_mat.mat[4] << "\t"
        << tf_mat.mat[5] << std::endl
        << "0.000\t0.000\t1.000" << std::endl;
    return out;
}

}; // namespace kelo::geometry_common

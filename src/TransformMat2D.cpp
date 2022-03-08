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
#include <geometry_common/Pose2D.h>
#include <geometry_common/Point2D.h>
#include <geometry_common/Polyline2D.h>
#include <geometry_common/Polygon2D.h>
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
    float x = stamped_transform.getOrigin().x();
    float y = stamped_transform.getOrigin().y();

    tf::Quaternion q = stamped_transform.getRotation();
    update(x, y, q.x(), q.y(), q.z(), q.w());
}

TransformMat2D::TransformMat2D(const Pose2D& pose)
{
    update(pose);
}

TransformMat2D::TransformMat2D(const TransformMat2D& tf_mat)
{
    update(tf_mat);
}

void TransformMat2D::update(float x, float y, float theta)
{
    mat_[2] = x;
    mat_[5] = y;
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
    for ( size_t i = 0; i < mat_.size(); i++ )
    {
        mat_[i] = tf_mat[i];
    }
}

TransformMat2D TransformMat2D::getInverse() const
{
    TransformMat2D inv_tf_mat(*this);
    inv_tf_mat.invert();
    return inv_tf_mat;
}

void TransformMat2D::invert()
{
    // taking transpose of rotation matrix part since
    // inverse(M) == transpose(M) : where M is a orthonormal rotation matrix
    float temp = mat_[1];
    mat_[1] = mat_[3];
    mat_[3] = temp;

    // - inverse(M) * T : where M is as above and T is translation vector
    float x = mat_[2];
    float y = mat_[5];
    mat_[2] = -((mat_[0] * x) + (mat_[1] * y));
    mat_[5] = -((mat_[3] * x) + (mat_[4] * y));
}

float TransformMat2D::getX() const
{
    return mat_[2];
}

float TransformMat2D::getY() const
{
    return mat_[5];
}

float TransformMat2D::getTheta() const
{
    return std::atan2(mat_[3], mat_[0]);
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
    rot_mat[0] = mat_[0];
    rot_mat[1] = mat_[1];
    rot_mat[2] = mat_[3];
    rot_mat[3] = mat_[4];
    return rot_mat;
}

Vector2D TransformMat2D::getTranslationVec() const
{
    return Vector2D(mat_[2], mat_[5]);
}

Pose2D TransformMat2D::getPose2D() const
{
    return Pose2D(mat_[2], mat_[5], getTheta());
}

void TransformMat2D::setX(float x)
{
    mat_[2] = x;
}

void TransformMat2D::setY(float y)
{
    mat_[5] = y;
}

void TransformMat2D::setTheta(float theta)
{
    mat_[0] = std::cos(theta);
    mat_[1] = -std::sin(theta);
    mat_[3] = std::sin(theta);
    mat_[4] = std::cos(theta);
}

void TransformMat2D::setQuaternion(float qx, float qy, float qz, float qw)
{
    float roll, pitch, yaw;
    Utils::getEulerFromQuaternion(qx, qy, qz, qw, roll, pitch, yaw);
    setTheta(yaw);
}

void TransformMat2D::transform(Point2D& point) const
{
    float temp_x = (mat_[0] * point.x) + (mat_[1] * point.y) + mat_[2];
    float temp_y = (mat_[3] * point.x) + (mat_[4] * point.y) + mat_[5];
    point.x = temp_x;
    point.y = temp_y;
}

void TransformMat2D::transform(Pose2D& pose) const
{
    TransformMat2D transformed_mat = (*this) * pose.getMat();
    pose.x = transformed_mat[2];
    pose.y = transformed_mat[5];
    pose.theta = std::atan2(transformed_mat[3], transformed_mat[0]);
}

void TransformMat2D::transform(Polyline2D& polyline) const
{
    for ( Point2D& vertex : polyline.vertices )
    {
        transform(vertex);
    }
}

TransformMat2D TransformMat2D::operator * (const TransformMat2D& tf_mat) const
{
    TransformMat2D result_tf_mat(*this);
    result_tf_mat *= tf_mat;
    return result_tf_mat;
}

TransformMat2D& TransformMat2D::operator *= (const TransformMat2D& tf_mat)
{
    float arr[6];
    arr[0] = (mat_[0] * tf_mat[0]) + (mat_[1] * tf_mat[3]);
    arr[1] = (mat_[0] * tf_mat[1]) + (mat_[1] * tf_mat[4]);
    arr[2] = (mat_[0] * tf_mat[2]) + (mat_[1] * tf_mat[5]) + mat_[2];
    arr[3] = (mat_[3] * tf_mat[0]) + (mat_[4] * tf_mat[3]);
    arr[4] = (mat_[3] * tf_mat[1]) + (mat_[4] * tf_mat[4]);
    arr[5] = (mat_[3] * tf_mat[2]) + (mat_[4] * tf_mat[5]) + mat_[5];
    for ( size_t i = 0; i < 6; i++ )
    {
        mat_[i] = arr[i];
    }
    return *this;
}

Point2D TransformMat2D::operator * (const Point2D& point) const
{
    Point2D transformed_point(point);
    transform(transformed_point);
    return transformed_point;
}

Pose2D TransformMat2D::operator * (const Pose2D& pose) const
{
    TransformMat2D transformed_mat = (*this) * pose.getMat();
    return transformed_mat.getPose2D();
}

Polyline2D TransformMat2D::operator * (const Polyline2D& polyline) const
{
    Polyline2D transformed_polyline(polyline);
    transform(transformed_polyline);
    return transformed_polyline;
}

Polygon2D TransformMat2D::operator * (const Polygon2D& polygon) const
{
    Polyline2D transformed_polygon(polygon);
    transform(transformed_polygon);
    return transformed_polygon;
}

const float& TransformMat2D::operator [] (unsigned int index) const
{
    return mat_[index];
}

std::ostream& operator << (std::ostream& out, const TransformMat2D& tf_mat)
{
    out << std::setprecision(3) << std::fixed;
    out << tf_mat.mat_[0] << "\t"
        << tf_mat.mat_[1] << "\t"
        << tf_mat.mat_[2] << std::endl
        << tf_mat.mat_[3] << "\t"
        << tf_mat.mat_[4] << "\t"
        << tf_mat.mat_[5] << std::endl
        << "0.000\t0.000\t1.000" << std::endl;
    return out;
}

}; // namespace kelo::geometry_common

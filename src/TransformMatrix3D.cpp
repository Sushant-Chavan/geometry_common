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
#include <geometry_common/TransformMatrix3D.h>

namespace kelo::geometry_common
{

TransformMatrix3D::TransformMatrix3D(
        float x, float y, float z, float roll, float pitch, float yaw)
{
    update(x, y, z, roll, pitch, yaw);
}

TransformMatrix3D::TransformMatrix3D(
        float x, float y, float z, float qx, float qy, float qz, float qw)
{
    update(x, y, z, qx, qy, qz, qw);
}

TransformMatrix3D::TransformMatrix3D(const tf::StampedTransform& stamped_transform)
{
    float x = stamped_transform.getOrigin().x();
    float y = stamped_transform.getOrigin().y();
    float z = stamped_transform.getOrigin().z();

    tf::Quaternion q = stamped_transform.getRotation();
    update(x, y, z, q.x(), q.y(), q.z(), q.w());
}

TransformMatrix3D::TransformMatrix3D(const TransformMatrix3D& tf_mat)
{
    update(tf_mat);
}

void TransformMatrix3D::update(
        float x, float y, float z, float roll, float pitch, float yaw)
{
    setX(x);
    setY(y);
    setZ(z);
    setRollPitchYaw(roll, pitch, yaw);
}

void TransformMatrix3D::update(
        float x, float y, float z, float qx, float qy, float qz, float qw)
{
    float roll, pitch, yaw;
    Utils::getEulerFromQuaternion(qx, qy, qz, qw, roll, pitch, yaw);
    update(x, y, z, roll, pitch, yaw);
}

void TransformMatrix3D::update(const TransformMatrix3D& tf_mat)
{
    for ( size_t i = 0; i < 12; i++ )
    {
        mat_[i] = tf_mat[i];
    }
}

TransformMatrix3D TransformMatrix3D::getInverse() const
{
    TransformMatrix3D inv_tf_mat(*this);
    inv_tf_mat.invert();
    return inv_tf_mat;
}

void TransformMatrix3D::invert()
{
    // taking transpose of rotation matrix part since
    // inverse(M) == transpose(M) : where M is a orthonormal rotation matrix
    for ( size_t i = 0; i < 2; i++ )
    {
        for ( size_t j = i+1; j < 3; j++ )
        {
            float temp = mat_[(i*4)+j];
            mat_[(i*4)+j] = mat_[(j*4)+i];
            mat_[(j*4)+i] = temp;
        }
    }

    // - inverse(M) * T : where M is as above and T is translation vector
    float x = mat_[3];
    float y = mat_[7];
    float z = mat_[11];
    mat_[3] = -((mat_[0] * x) + (mat_[1] * y) + (mat_[2] * z));
    mat_[7] = -((mat_[4] * x) + (mat_[5] * y) + (mat_[6] * z));
    mat_[11] = -((mat_[8] * x) + (mat_[9] * y) + (mat_[10] * z));
}

float TransformMatrix3D::getX() const
{
    return mat_[3];
}

float TransformMatrix3D::getY() const
{
    return mat_[7];
}

float TransformMatrix3D::getZ() const
{
    return mat_[11];
}

float TransformMatrix3D::getRoll() const
{
    return std::atan2(mat_[9], mat_[10]);
}

float TransformMatrix3D::getPitch() const
{
    return std::atan2(-mat_[8], std::sqrt(pow(mat_[9], 2) + pow(mat_[10], 2)));
}

float TransformMatrix3D::getYaw() const
{
    return std::atan2(mat_[4], mat_[0]);
}

std::array<float, 4> TransformMatrix3D::getQuaternion() const
{
    std::array<float, 4> q;
    Utils::getQuaternionFromEuler(getRoll(), getPitch(), getYaw(),
                                  q[0], q[1], q[2], q[3]);
    return q;
}

std::array<float, 9> TransformMatrix3D::getRotationMat() const
{
    std::array<float, 9> rot_mat;
    for ( size_t i = 0; i < 3; i++ )
    {
        for ( size_t j = 0; j < 3; j++ )
        {
            rot_mat[(i*3)+j] = mat_[(i*4) + j];
        }
    }
    return rot_mat;
}

Vector3D TransformMatrix3D::getTranslationVec() const
{
    return Vector3D(mat_[3], mat_[7], mat_[11]);
}

void TransformMatrix3D::setX(float x)
{
    mat_[3] = x;
}

void TransformMatrix3D::setY(float y)
{
    mat_[7] = y;
}

void TransformMatrix3D::setZ(float z)
{
    mat_[11] = z;
}

void TransformMatrix3D::setRoll(float roll)
{
    setRollPitchYaw(roll, getPitch(), getYaw());
}

void TransformMatrix3D::setPitch(float pitch)
{
    setRollPitchYaw(getRoll(), pitch, getYaw());
}

void TransformMatrix3D::setYaw(float yaw)
{
    setRollPitchYaw(getRoll(), getPitch(), yaw);
}

void TransformMatrix3D::setRollPitchYaw(float roll, float pitch, float yaw)
{
    mat_[0] = (std::cos(yaw) * std::cos(pitch));
    mat_[1] = (std::cos(yaw) * std::sin(pitch) * std::sin(roll)) -
              (std::sin(yaw) * std::cos(roll));
    mat_[2] = (std::cos(yaw) * std::sin(pitch) * std::cos(roll)) +
              (std::sin(yaw) * std::sin(roll));
    mat_[4] = (std::sin(yaw) * std::cos(pitch));
    mat_[5] = (std::sin(yaw) * std::sin(pitch) * std::sin(roll)) +
              (std::cos(yaw) * std::cos(roll));
    mat_[6] = (std::sin(yaw) * std::sin(pitch) * std::cos(roll)) -
              (std::cos(yaw) * std::sin(roll));
    mat_[8] = -std::sin(pitch);
    mat_[9] = std::cos(pitch) * std::sin(roll);
    mat_[10] = std::cos(pitch) * std::cos(roll);
}

void TransformMatrix3D::setQuaternion(float qx, float qy, float qz, float qw)
{
    float roll, pitch, yaw;
    Utils::getEulerFromQuaternion(qx, qy, qz, qw, roll, pitch, yaw);
    setRollPitchYaw(roll, pitch, yaw);
}

void TransformMatrix3D::transform(Point3D& point) const
{
    float temp_x = (mat_[0] * point.x) + (mat_[1] * point.y) +
                   (mat_[2] * point.z) + mat_[3];
    float temp_y = (mat_[4] * point.x) + (mat_[5] * point.y) +
                   (mat_[6] * point.z) + mat_[7];
    float temp_z = (mat_[8] * point.x) + (mat_[9] * point.y) +
                   (mat_[10] * point.z) + mat_[11];
    point.x = temp_x;
    point.y = temp_y;
    point.z = temp_z;
}

TransformMatrix3D TransformMatrix3D::operator * (const TransformMatrix3D& tf_mat) const
{
    TransformMatrix3D result_tf_mat(*this);
    result_tf_mat *= tf_mat;
    return result_tf_mat;
}

TransformMatrix3D& TransformMatrix3D::operator *= (const TransformMatrix3D& tf_mat)
{
    float arr[12]; // temporary array

    // apply rotation to tf_mat by multipling 3x3 rot mat part of mat_ with tf_mat
    for ( size_t i = 0; i < 3; i++ )
    {
        for ( size_t j = 0; j < 4; j++ )
        {
            for ( size_t k = 0; k < 3; k++ )
            {
                arr[i*4 + j] += mat_[i*4 + k] * tf_mat[k*4 + j];
            }
        }
    }

    // apply translation to tf_mat by adding 3x1 translation part of mat_
    arr[3] += mat_[3];
    arr[7] += mat_[7];
    arr[11] += mat_[11];

    //copy temp arr to mat_
    for ( size_t i = 0; i < 12; i++ )
    {
        mat_[i] = arr[i];
    }
    return *this;
}

Point3D TransformMatrix3D::operator * (const Point3D& point) const
{
    Point3D transformed_point(point);
    transform(transformed_point);
    return transformed_point;
}

const float& TransformMatrix3D::operator [] (unsigned int index) const
{
    return mat_[index];
}

std::ostream& operator << (std::ostream& out, const TransformMatrix3D& tf_mat)
{
    out << std::setprecision(3) << std::fixed;
    out << tf_mat.mat_[0] << "\t"
        << tf_mat.mat_[1] << "\t"
        << tf_mat.mat_[2] << "\t"
        << tf_mat.mat_[3] << std::endl
        << tf_mat.mat_[4] << "\t"
        << tf_mat.mat_[5] << "\t"
        << tf_mat.mat_[6] << "\t"
        << tf_mat.mat_[7] << std::endl
        << tf_mat.mat_[8] << "\t"
        << tf_mat.mat_[9] << "\t"
        << tf_mat.mat_[10] << "\t"
        << tf_mat.mat_[11] << std::endl
        << "0.000\t0.000\t0.000\t1.000" << std::endl;
    return out;
}

}; // namespace kelo::geometry_common

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
#include <geometry_common/Attitude2D.h>
#include <geometry_common/Utils.h>

namespace kelo::geometry_common
{

Attitude2D Attitude2D::operator + (const Attitude2D& attitude) const
{
    Attitude2D sum;
    sum.x = x + attitude.x;
    sum.y = y + attitude.y;
    sum.theta = theta + attitude.theta;
    return sum;
}

Attitude2D Attitude2D::operator - (const Attitude2D& attitude) const
{
    Attitude2D diff;
    diff.x = x - attitude.x;
    diff.y = y - attitude.y;
    diff.theta = theta - attitude.theta;
    return diff;
}

Attitude2D Attitude2D::operator * (float scalar) const
{
    Attitude2D scaled;
    scaled.x = x * scalar;
    scaled.y = y * scalar;
    scaled.theta = theta * scalar;
    return scaled;
}

bool Attitude2D::operator == (const Attitude2D& attitude) const
{
    Attitude2D diff = *this - attitude;
    return ( pow(diff.x, 2) + pow(diff.y, 2) + pow(diff.theta, 2) < 1e-6f );
}

std::ostream& operator << (std::ostream& out, const Attitude2D& attitude)
{
    out << "<x: " << attitude.x
        << ", y: " << attitude.y
        << ", theta: " << attitude.theta
        << ">";
    return out;
}

} // namespace kelo::geometry_common

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

#ifndef KELO_GEOMETRY_COMMON_ATTITUDE_2D_H
#define KELO_GEOMETRY_COMMON_ATTITUDE_2D_H

#include <iostream>
#include <memory>

namespace kelo::geometry_common
{

// Forward declaration 
class Pose2D;

/**
 * @brief 
 * 
 */
class Attitude2D
{
    public:
        float x, y, theta;

        using Ptr = std::shared_ptr<Attitude2D>;
        using ConstPtr = std::shared_ptr<const Attitude2D>;

        /**
         * @brief
         * 
         * @param _x 
         * @param _y 
         * @param _theta 
         */
        Attitude2D(float _x = 0.0f, float _y = 0.0f, float _theta = 0.0f):
            x(_x), y(_y), theta(_theta) {}

        /**
         * @brief
         * 
         * @param pose 
         */
        Attitude2D(const Attitude2D& attitude):
            x(attitude.x), y(attitude.y), theta(attitude.theta) {}

        /**
         * @brief
         * 
         */
        virtual ~Attitude2D() {}

        /**
         * @brief 
         *
         * @param attitude
         * @return 
         */
        Attitude2D operator + (const Attitude2D& attitude) const;

        /**
         * @brief 
         * 
         * @param attitude 
         * @return Attitude2D 
         */
        Attitude2D operator - (const Attitude2D& attitude) const;

        /**
         * @brief 
         * 
         * @param scalar 
         * @return Attitude2D 
         */
        Attitude2D operator * (float scalar) const;

        /**
         * @brief 
         * 
         * @param attitude 
         * @return bool 
         */
        bool operator == (const Attitude2D& attitude) const;

        /**
         * @brief 
         * 
         * @param out 
         * @param attitude 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (
                std::ostream& out,
                const Attitude2D& attitude);
};

using Velocity2D = Attitude2D;
using Acceleration2D = Attitude2D;

} // namespace kelo::geometry_common

#endif // KELO_GEOMETRY_COMMON_POSE_2D_H

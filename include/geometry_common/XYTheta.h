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

#ifndef KELO_GEOMETRY_COMMON_X_Y_THETA_H
#define KELO_GEOMETRY_COMMON_X_Y_THETA_H

#include <iostream>
#include <memory>

namespace kelo::geometry_common
{

/**
 * @brief 
 * 
 */
class XYTheta
{
    public:
        float x{0.0f}, y{0.0f}, theta{0.0f};

        using Ptr = std::shared_ptr<XYTheta>;
        using ConstPtr = std::shared_ptr<const XYTheta>;

        /**
         * @brief
         * 
         * @param _x 
         * @param _y 
         * @param _theta 
         */
        XYTheta(float _x = 0.0f, float _y = 0.0f, float _theta = 0.0f):
            x(_x), y(_y), theta(_theta) {}

        /**
         * @brief
         * 
         * @param pose 
         */
        XYTheta(const XYTheta& x_y_theta):
            x(x_y_theta.x), y(x_y_theta.y), theta(x_y_theta.theta) {}

        /**
         * @brief
         * 
         */
        virtual ~XYTheta() {}

        /**
         * @brief 
         * 
         * @param other 
         * @return XYTheta& 
         */
        XYTheta& operator = (const XYTheta& other);

        /**
         * @brief 
         *
         * @param x_y_theta
         * @return 
         */
        XYTheta operator + (const XYTheta& x_y_theta) const;

        /**
         * @brief 
         * 
         * @param x_y_theta 
         * @return XYTheta 
         */
        XYTheta operator - (const XYTheta& x_y_theta) const;

        /**
         * @brief 
         * 
         * @param scalar 
         * @return XYTheta 
         */
        XYTheta operator * (float scalar) const;

        /**
         * @brief 
         * 
         * @param x_y_theta 
         * @return bool 
         */
        bool operator == (const XYTheta& x_y_theta) const;

        /**
         * @brief 
         * 
         * @param out 
         * @param x_y_theta 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (
                std::ostream& out,
                const XYTheta& x_y_theta);
};

using Velocity2D = XYTheta;
using Acceleration2D = XYTheta;

} // namespace kelo::geometry_common

#endif // KELO_GEOMETRY_COMMON_X_Y_THETA_H

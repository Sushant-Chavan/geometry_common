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

#ifndef KELO_GEOMETRY_COMMON_BOX_H
#define KELO_GEOMETRY_COMMON_BOX_H

#include <yaml-cpp/yaml.h>

#include <visualization_msgs/Marker.h>
#include <geometry_common/Point3D.h>

namespace kelo
{
namespace geometry_common
{

/**
 * @brief 
 * 
 */
class Box
{
    public:
        float min_x, max_x, min_y, max_y, min_z, max_z;
        
        using Ptr = std::shared_ptr<Box>;
        using ConstPtr = std::shared_ptr<const Box>;

        /**
         * @brief
         * 
         * @param _min_x 
         * @param _max_x 
         * @param _min_y 
         * @param _max_y 
         * @param _min_z 
         * @param _max_z 
         */
        Box(float _min_x = 0.0f, float _max_x = 0.0f,
            float _min_y = 0.0f, float _max_y = 0.0f,
            float _min_z = 0.0f, float _max_z = 0.0f):
            min_x(_min_x), max_x(_max_x),
            min_y(_min_y), max_y(_max_y),
            min_z(_min_z), max_z(_max_z) {};

        /**
         * @brief
         * 
         * @param box 
         */
        Box(const Box& box):
            min_x(box.min_x), max_x(box.max_x),
            min_y(box.min_y), max_y(box.max_y),
            min_z(box.min_z), max_z(box.max_z) {};

        /**
         * @brief
         * 
         */
        virtual ~Box() {};

        /**
         * @brief 
         * 
         * @param yaml_box_params 
         * @param box 
         * @return bool 
         */
        static bool initialiseBoxFromYAML(const YAML::Node& yaml_box_params, Box& box);

        /**
         * @brief
         * 
         * @param frame 
         * @param red 
         * @param green 
         * @param blue 
         * @param alpha 
         * @return visualization_msgs::Marker 
         */
        visualization_msgs::Marker asMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f) const;

        /**
         * @brief 
         * 
         * @param p 
         * @return bool 
         */
        bool containsPoint(const Point3D& p) const;

        /**
         * @brief 
         * 
         * @param other 
         * @return Box& 
         */
        Box& operator = (const Box& other);

        /**
         * @brief 
         *
         * @param box
         * @return bool
         */
        bool operator == (const Box& box) const;

        /**
         * @brief 
         * 
         * @param out 
         * @param box 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (std::ostream& out, const Box& box);
};

} // namespace geometry_common
} // namespace kelo
#endif // KELO_GEOMETRY_COMMON_BOX_H

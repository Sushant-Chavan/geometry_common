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

#include <geometry_common/Box.h>

namespace kelo
{
namespace geometry_common
{

bool Box::initialiseBoxFromYAML(const YAML::Node& yaml_box_params, Box& box)
{
    if ( !yaml_box_params["min_x"] || !yaml_box_params["max_x"] ||
         !yaml_box_params["min_y"] || !yaml_box_params["max_y"] ||
         !yaml_box_params["min_z"] || !yaml_box_params["max_z"] )
    {
        std::cout << std::endl << std::endl;
        std::cerr << "Box params should contain the following:" << std::endl;
        std::cerr << "\t- min_x" << std::endl;
        std::cerr << "\t- max_x" << std::endl;
        std::cerr << "\t- min_y" << std::endl;
        std::cerr << "\t- max_y" << std::endl;
        std::cerr << "\t- min_z" << std::endl;
        std::cerr << "\t- max_z" << std::endl;
        std::cout << std::endl << std::endl;
        return false;
    }
    box.min_x = yaml_box_params["min_x"].as<float>();
    box.max_x = yaml_box_params["max_x"].as<float>();
    box.min_y = yaml_box_params["min_y"].as<float>();
    box.max_y = yaml_box_params["max_y"].as<float>();
    box.min_z = yaml_box_params["min_z"].as<float>();
    box.max_z = yaml_box_params["max_z"].as<float>();
    return true;
}

visualization_msgs::Marker Box::asMarker(const std::string& frame,
        float red, float green, float blue, float alpha) const
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = max_x-min_x;
    marker.scale.y = max_y-min_y;
    marker.scale.z = max_z-min_z;
    marker.pose.position.x = (max_x+min_x)/2;
    marker.pose.position.y = (max_y+min_y)/2;
    marker.pose.position.z = (max_z+min_z)/2;
    marker.pose.orientation.w = 1.0f;
    return marker;
}

bool Box::containsPoint(const Point3D& p) const
{
    return ( p.x >= min_x && p.x <= max_x &&
             p.y >= min_y && p.y <= max_y &&
             p.z >= min_z && p.z <= max_z );
}

Box& Box::operator = (const Box& other)
{
    min_x = other.min_x;
    max_x = other.max_x;
    min_y = other.min_y;
    max_y = other.max_y;
    min_z = other.min_z;
    max_z = other.max_z;
    return *this;
}

bool Box::operator == (const Box& box) const
{
    return ( std::fabs(min_x - box.min_x) < 1e-3f &&
             std::fabs(max_x - box.max_x) < 1e-3f &&
             std::fabs(min_y - box.min_y) < 1e-3f &&
             std::fabs(max_y - box.max_y) < 1e-3f &&
             std::fabs(min_z - box.min_z) < 1e-3f &&
             std::fabs(max_z - box.max_z) < 1e-3f );
}

std::ostream& operator << (std::ostream& out, const Box& box)
{
    out <<  "<min_x: " << box.min_x
        << ", max_x: " << box.max_x
        << ", min_y: " << box.min_y
        << ", max_y: " << box.max_y
        << ", min_z: " << box.min_z
        << ", max_z: " << box.max_z
        << ">";
    return out;
}

} // namespace geometry_common
} // namespace kelo

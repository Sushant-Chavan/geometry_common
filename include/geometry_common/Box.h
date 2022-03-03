#ifndef KELO_GEOMETRY_COMMON_BOX_H
#define KELO_GEOMETRY_COMMON_BOX_H

#include <yaml-cpp/yaml.h>

#include <visualization_msgs/Marker.h>
#include <geometry_common/Point3D.h>

namespace kelo::geometry_common
{

/**
 * @brief 
 * 
 */
class Box
{
    public:
        float min_x, max_x, min_y, max_y, min_z, max_z;

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
        visualization_msgs::Marker getMarker(
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
        bool isPointInsideBox(const Point3D& p) const;

        /**
         * @brief 
         * 
         * @param out 
         * @param box 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (std::ostream& out, const Box& box);
};

}; // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_BOX_H

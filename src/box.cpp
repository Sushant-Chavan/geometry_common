#include <geometry_common/box.h>

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

bool Box::isPointInsideBox(const Point& p) const
{
    return ( p.x >= min_x && p.x <= max_x &&
            p.y >= min_y && p.y <= max_y &&
            p.z >= min_z && p.z <= max_z );
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

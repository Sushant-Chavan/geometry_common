#include <geometry_common/Point3d.h>
#include <geometry_common/Polygon2d.h>

namespace kelo::geometry_common
{

Polygon2D::~Polygon2D() {}

visualization_msgs::Marker Polygon2D::getMarker(const std::string& frame,
                                                float red, float green,
                                                float blue, float alpha,
                                                float line_width,
                                                double z) const
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    // marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = line_width;
    marker.pose.orientation.w = 1.0f;
    if (!corners.empty())
    {
        for (const auto& corner : corners)
        {
            marker.points.push_back(Point3D(corner, z).getPoint());
        }
        // Repeat first point again to close the polygon loop
        marker.points.push_back(Point3D(corners[0], z).getPoint());
    }
    return marker;
}

std::ostream& operator<<(std::ostream& out, const Polygon2D& polygon)
{
    out << "Polygon corners:" << std::endl;
    for (const auto& corner : polygon.corners)
    {
        out << corner;
    }
    return out;
}

} // namespace kelo::geometry_common

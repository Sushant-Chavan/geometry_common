#include <geometry_common/Point3D.h>
#include <geometry_common/Polyline2D.h>

namespace kelo::geometry_common
{

void Polyline2D::transform(const std::vector<float>& tf_mat)
{
    for ( Point2D& vertex : vertices )
    {
        vertex.transform(tf_mat);
    }
}

void Polyline2D::transform(const Pose2D& tf)
{
    for ( Point2D& vertex : vertices )
    {
        vertex.transform(tf);
    }
}

Polyline2D Polyline2D::getTransformedPolyline(const std::vector<float>& tf_mat) const
{
    Polyline2D transformed_poly(*this);
    transformed_poly.transform(tf_mat);
    return transformed_poly;
}

Polyline2D Polyline2D::getTransformedPolyline(const Pose2D& tf) const
{
    Polyline2D transformed_poly(*this);
    transformed_poly.transform(tf);
    return transformed_poly;
}

visualization_msgs::Marker Polyline2D::getMarker(const std::string& frame,
        float red, float green, float blue, float alpha, float line_width,
        float z) const
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
    if (!vertices.empty())
    {
        marker.points.reserve(vertices.size());
        for ( const Point2D& vertex : vertices )
        {
            marker.points.push_back(Point3D(vertex, z).getPoint());
        }
    }
    return marker;
}

std::ostream& operator<<(std::ostream& out, const Polyline2D& polyline)
{
    out << "Polyline vertices:" << std::endl;
    for (const auto& vertex : polyline.vertices)
    {
        out << vertex;
    }
    return out;
}

} // namespace kelo::geometry_common

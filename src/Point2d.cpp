#include <geometry_common/Point2d.h>

namespace kelo::geometry_common
{

Point2D Point2D::normalise() const
{
    float mag = magnitude();
    if ( mag == 0 )
    {
        return Point2D();
    }
    return Point2D(x/mag, y/mag);
}

visualization_msgs::Marker Point2D::getMarker(const std::string& frame,
        float red, float green, float blue, float alpha, float diameter, double z) const
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = diameter;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0f;
    return marker;
}

Point2D operator - (const Point2D& p1, const Point2D& p2)
{
    Point2D diff;
    diff.x = p1.x - p2.x;
    diff.y = p1.y - p2.y;
    return diff;
}

Point2D operator + (const Point2D& p1, const Point2D& p2)
{
    Point2D sum;
    sum.x = p1.x + p2.x;
    sum.y = p1.y + p2.y;
    return sum;
}

Point2D operator * (const Point2D& p1, float scalar)
{
    Point2D scaled;
    scaled.x = p1.x * scalar;
    scaled.y = p1.y * scalar;
    return scaled;
}

std::ostream& operator << (std::ostream& out, const Point2D& point)
{
    out << "<x: " << point.x << ", y: " << point.y << ">";
    return out;
}

bool operator == (const Point2D& p1, const Point2D& p2)
{
    return ( p1.getCartDist(p2) < 1e-3f );
}

} // namespace kelo::geometry_common

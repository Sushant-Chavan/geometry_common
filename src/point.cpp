#include <geometry_common/point.h>

namespace geometry_common
{

geometry_msgs::Point Point::getPoint() const
{
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

geometry_msgs::Point32 Point::getPoint32() const
{
    geometry_msgs::Point32 point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

geometry_msgs::PointStamped Point::getPointStamped(const std::string& frame) const
{
    geometry_msgs::PointStamped point;
    point.header.stamp = ros::Time::now();
    point.header.frame_id = frame;
    point.point = getPoint();
    return point;
}

Point Point::normalise() const
{
    float mag = magnitude();
    if ( mag == 0 )
    {
        return Point();
    }
    return Point(x/mag, y/mag, z/mag);
}

visualization_msgs::Marker Point::getMarker(const std::string& frame,
        float red, float green, float blue, float alpha, float diameter) const
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

Point operator - (const Point& p1, const Point& p2)
{
    Point diff;
    diff.x = p1.x - p2.x;
    diff.y = p1.y - p2.y;
    diff.z = p1.z - p2.z;
    return diff;
}

Point operator + (const Point& p1, const Point& p2)
{
    Point sum;
    sum.x = p1.x + p2.x;
    sum.y = p1.y + p2.y;
    sum.z = p1.z + p2.z;
    return sum;
}

Point operator * (const Point& p1, float scalar)
{
    Point scaled;
    scaled.x = p1.x * scalar;
    scaled.y = p1.y * scalar;
    scaled.z = p1.z * scalar;
    return scaled;
}

std::ostream& operator << (std::ostream& out, const Point& point)
{
    out << "<x: " << point.x << ", y: " << point.y << ", z: " << point.z << ">";
    return out;
}

bool operator == (const Point& p1, const Point& p2)
{
    return ( p1.getCartDist(p2) < 0.001f );
}

} // namespace geometry_common

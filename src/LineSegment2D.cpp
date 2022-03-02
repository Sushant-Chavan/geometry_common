#include <geometry_common/Point3D.h>
#include <geometry_common/LineSegment2D.h>

namespace kelo::geometry_common
{

LineSegment2D::~LineSegment2D()
{
}

float LineSegment2D::getAngle() const
{
    float dx = end.x - start.x;
    float dy = end.y - start.y;
    return atan2(dy, dx);
}

float LineSegment2D::getLength() const
{
    return start.getCartDist(end);
}

float LineSegment2D::getSlope() const
{
    float dx = end.x - start.x;
    float dy = end.y - start.y;
    if ( fabs(dx) < 1e-6f )
    {
        dx = 1e-6f;
    }
    return dy/dx;
}

float LineSegment2D::getConstant() const
{
    float m = getSlope();
    return start.y - (m * start.x);
}

Point2D LineSegment2D::getCenter() const
{
    return (start + end) * 0.5f;
}

Point2D LineSegment2D::getUnitVector() const
{
    return (end - start) * (1.0f/getLength());
}

visualization_msgs::Marker LineSegment2D::getMarker(const std::string& frame,
        float red, float green, float blue, float alpha, float line_width) const
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    // marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = line_width;
    marker.pose.orientation.w = 1.0f;
    marker.points.push_back(Point3D(start).getPoint());
    marker.points.push_back(Point3D(end).getPoint());
    return marker;
}

std::ostream& operator << (std::ostream &out, const LineSegment2D& line_segment)
{
    out << "start: " << line_segment.start << std::endl;
    out << "end: " << line_segment.end << std::endl;
    return out;
}

} // namespace kelo::geometry_common

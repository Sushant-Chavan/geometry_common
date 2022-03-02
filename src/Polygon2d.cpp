#include <geometry_common/Point3d.h>
#include <geometry_common/Polygon2d.h>

namespace kelo::geometry_common
{

Polygon2D::~Polygon2D() {}

bool Polygon2D::containsPoint(const Point2D& point) const
{
    /* 
     * Source: https://stackoverflow.com/a/2922778/10460994
     */
    size_t i, j, counter = 0;
    for (i = 0, j = corners.size()-1; i < corners.size(); j = i++)
    {
        if ( ((corners[i].y > point.y) != (corners[j].y > point.y)) &&
             (point.x < (corners[j].x - corners[i].x) * (point.y - corners[i].y) / (corners[j].y - corners[i].y) + corners[i].x) )
        {
            counter++;
        }
    }
    return (counter % 2 == 1);
}

Point2D Polygon2D::getMeanPoint() const
{
    Point2D mean;
    if (!corners.empty())
    {
        for (const auto& corner: corners)
        {
            mean = mean + corner;
        }
        mean = mean * (1.0f/corners.size());
    }
    return mean;
}

float Polygon2D::getArea() const
{
    float area = 0.0f;
    size_t i, j;
    for ( i = 0; i < corners.size(); i++ )
    {
        j = (i+1) % corners.size();
        area += (corners[i].x * corners[j].y) - (corners[i].y * corners[j].x);
    }
    return area/2;
}

bool Polygon2D::isConvex() const
{
    if ( corners.size() <= 2 )
    {
        return true;
    }

    Point2D meanPt = getMeanPoint();
    if ( !containsPoint(meanPt) )
    {
        return false;
    }

    for ( size_t i = 0; i < corners.size(); i++ )
    {
        Point3D p1(corners[i]);
        Point3D p2(corners[(i+2)%corners.size()]);
        meanPt = (p1 + p2) * 0.5f;
        if ( !containsPoint(meanPt) )
        {
            return false;
        }
    }
    return true;
}

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

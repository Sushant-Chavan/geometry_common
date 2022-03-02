#include <geometry_common/Point3D.h>
#include <geometry_common/Polygon2D.h>

namespace kelo::geometry_common
{

Polygon2D::~Polygon2D() {}

bool Polygon2D::containsPoint(const Point2D& point) const
{
    /* 
     * Source: https://stackoverflow.com/a/2922778/10460994
     */
    size_t i, j, counter = 0;
    for (i = 0, j = vertices.size()-1; i < vertices.size(); j = i++)
    {
        const Point2D& currVert = vertices[i];
        const Point2D& prevVert = vertices[j];
        if ( ((currVert.y > point.y) != (prevVert.y > point.y)) &&
             (point.x < (prevVert.x - currVert.x) * (point.y - currVert.y) / (prevVert.y - currVert.y) + currVert.x) )
        {
            counter++;
        }
    }
    return (counter % 2 == 1);
}

Point2D Polygon2D::getMeanPoint() const
{
    Point2D mean;
    if (!vertices.empty())
    {
        for (const auto& corner: vertices)
        {
            mean = mean + corner;
        }
        mean = mean * (1.0f/vertices.size());
    }
    return mean;
}

float Polygon2D::getArea() const
{
    float area = 0.0f;
    size_t i, j;
    for ( i = 0; i < vertices.size(); i++ )
    {
        j = (i+1) % vertices.size();
        area += (vertices[i].x * vertices[j].y) - (vertices[i].y * vertices[j].x);
    }
    return area/2;
}

bool Polygon2D::isConvex() const
{
    if ( vertices.size() <= 2 )
    {
        return true;
    }

    Point2D meanPt = getMeanPoint();
    if ( !containsPoint(meanPt) )
    {
        return false;
    }

    for ( size_t i = 0; i < vertices.size(); i++ )
    {
        Point2D p1(vertices[i]);
        Point2D p2(vertices[(i+2)%vertices.size()]);
        meanPt = (p1 + p2) * 0.5f;
        if ( !containsPoint(meanPt) )
        {
            return false;
        }
    }
    return true;
}

visualization_msgs::Marker Polygon2D::getMarker(const std::string& frame,
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
        for (const auto& corner : vertices)
        {
            marker.points.push_back(Point3D(corner, z).getPoint());
        }
        // Repeat first point again to close the polygon loop
        marker.points.push_back(Point3D(vertices[0], z).getPoint());
    }
    return marker;
}

std::ostream& operator<<(std::ostream& out, const Polygon2D& polygon)
{
    out << "Polygon corners:" << std::endl;
    for (const auto& corner : polygon.vertices)
    {
        out << corner;
    }
    return out;
}

} // namespace kelo::geometry_common

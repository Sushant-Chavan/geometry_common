#include <geometry_common/Utils.h>
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
    for ( i = 0, j = vertices.size() - 1; i < vertices.size(); j = i++ )
    {
        const Point2D& currVert = vertices[i];
        const Point2D& prevVert = vertices[j];
        if ( ((currVert.y > point.y) != (prevVert.y > point.y)) &&
             (point.x < (prevVert.x - currVert.x) * (point.y - currVert.y) / (prevVert.y - currVert.y) + currVert.x) )
        {
            counter++;
        }
    }
    return ( counter % 2 == 1 );
}

bool Polygon2D::containsAnyPoint(const PointVec2D& points) const
{
    for ( const Point2D& pt: points )
    {
        if ( containsPoint(pt) )
        {
            return true;
        }
    }
    return false; 
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

Polygon2D Polygon2D::calcConvexHullOfPolygons(
        const Polygon2D& polygon_a,
        const Polygon2D& polygon_b)
{
    /**
     * source: https://en.wikipedia.org/wiki/Graham_scan#Pseudocode
     */

    /* aggregate all points */
    PointVec2D pts;
    pts.reserve(pts.size() + polygon_a.vertices.size() + polygon_b.vertices.size());
    pts.insert(pts.end(), polygon_a.vertices.begin(), polygon_a.vertices.end());
    pts.insert(pts.end(), polygon_b.vertices.begin(), polygon_b.vertices.end());
    if ( pts.size() < 3 )
    {
        return pts;
    }

    /* find the lowest left most point */
    Point2D lower_left_pt(pts[0]);
    for ( size_t i = 0; i < pts.size(); i++ )
    {
        if ( pts[i].y < lower_left_pt.y )
        {
            lower_left_pt = pts[i];
        }
        else if ( pts[i].y == lower_left_pt.y && pts[i].x < lower_left_pt.x )
        {
            lower_left_pt = pts[i];
        }
    }

    /* sort points in increasing order of angle they and lower_left_pt makes with X axis */
    std::sort(pts.begin(), pts.end(),
              [&lower_left_pt](const Point2D& a, const Point2D& b)
              {
                  return std::atan2(a.y - lower_left_pt.y, a.x - lower_left_pt.x)
                       < std::atan2(b.y - lower_left_pt.y, b.x - lower_left_pt.x);
              });

    /* walk along pts and remove points that form non counter clockwise turn */
    PointVec2D convex_hull;
    for ( Point2D& p : pts )
    {
        while ( convex_hull.size() > 1 )
        {
            PointVec2D::const_iterator it = convex_hull.end();
            float angle = Utils::getAngleBetweenPoints(p, *(it-1), *(it-2));
            if ( angle > 0 ) // counter clockwise turn is allowed
            {
                break;
            }
            convex_hull.pop_back();
        }
        convex_hull.push_back(p);
    }
    return Polygon2D(convex_hull);
}

void Polygon2D::transform(const std::vector<float>& tf_mat)
{
    for ( Point2D& vert : vertices )
    {
        vert.transform(tf_mat);
    }
}

void Polygon2D::transform(const Pose2D& tf)
{
    for ( Point2D& vert : vertices )
    {
        vert.transform(tf);
    }
}

Polygon2D Polygon2D::getTransformedPolygon(const std::vector<float>& tf_mat) const
{
    Polygon2D transformed_poly(*this);
    transformed_poly.transform(tf_mat);
    return transformed_poly;
}

Polygon2D Polygon2D::getTransformedPolygon(const Pose2D& tf) const
{
    Polygon2D transformed_poly(*this);
    transformed_poly.transform(tf);
    return transformed_poly;
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
        for ( const Point2D& corner : vertices )
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

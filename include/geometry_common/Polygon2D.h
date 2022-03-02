#ifndef KELO_GEOMETRY_COMMON_POLYGON_H
#define KELO_GEOMETRY_COMMON_POLYGON_H

#include <visualization_msgs/Marker.h>

#include <geometry_common/Point2D.h>

namespace kelo::geometry_common
{

class Polygon2D
{
    public:
        PointVec2D vertices;

        Polygon2D() = default;

        Polygon2D(const Polygon2D& polygon):
            vertices(polygon.vertices) {}

        Polygon2D(const PointVec2D& verts):
            vertices(verts) {}

        virtual ~Polygon2D();

        bool containsPoint(const Point2D& point) const;

        Point2D getMeanPoint() const;

        /*
         * Note: area can be negative (the sign informs if the polygon is
         * clockwise or not)
         */
        float getArea() const;

        bool isConvex() const;

        visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.1f,
                float z = 0.0f) const;

        friend std::ostream& operator << (std::ostream &out, const Polygon2D& line_segment);
};

} // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_POLYGON_H

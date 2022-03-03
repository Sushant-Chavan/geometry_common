#ifndef KELO_GEOMETRY_COMMON_POLYGON_2D_H
#define KELO_GEOMETRY_COMMON_POLYGON_2D_H

#include <visualization_msgs/Marker.h>

#include <geometry_common/Polyline2D.h>

namespace kelo::geometry_common
{

class Polygon2D : public Polyline2D
{
    public:
        using Ptr = std::shared_ptr<Polygon2D>;
        using ConstPtr = std::shared_ptr<const Polygon2D>;

        Polygon2D():
            Polyline2D() {}

        Polygon2D(const Polygon2D& polygon):
            Polyline2D(polygon) {}

        Polygon2D(const Polyline2D& polyline):
            Polyline2D(polyline) {}

        Polygon2D(const PointVec2D& verts):
            Polyline2D(verts) {}

        virtual ~Polygon2D() {}

        bool containsPoint(const Point2D& point) const;

        bool containsAnyPoint(const PointVec2D& points) const;

        Point2D getMeanPoint() const;

        /*
         * Note: area can be negative (the sign informs if the polygon is
         * clockwise or not)
         */
        float getArea() const;

        bool isConvex() const;

        /**
         * @brief Find convex hull from the union of 2 polygons.
         * 
         * @param polygon_a 
         * @param polygon_b 
         * @return Polygon2D 
         */
        static Polygon2D calcConvexHullOfPolygons(
                const Polygon2D& polygon_a,
                const Polygon2D& polygon_b);

        Polygon2D getTransformedPolygon(const std::vector<float>& tf_mat) const;

        Polygon2D getTransformedPolygon(const Pose2D& tf) const;

        visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.1f,
                float z = 0.0f) const override;

        friend std::ostream& operator << (std::ostream &out, const Polygon2D& polygon);
};

} // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_POLYGON_2D_H

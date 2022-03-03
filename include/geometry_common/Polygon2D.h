#ifndef KELO_GEOMETRY_COMMON_POLYGON_2D_H
#define KELO_GEOMETRY_COMMON_POLYGON_2D_H

#include <visualization_msgs/Marker.h>

#include <geometry_common/Polyline2D.h>

namespace kelo::geometry_common
{

/**
 * @brief 
 * 
 */
class Polygon2D : public Polyline2D
{
    public:
        /**
         * @brief
         * 
         */
        Polygon2D():
            Polyline2D() {}

        /**
         * @brief
         * 
         * @param polygon 
         */
        Polygon2D(const Polygon2D& polygon):
            Polyline2D(polygon) {}

        /**
         * @brief
         * 
         * @param polyline 
         */
        Polygon2D(const Polyline2D& polyline):
            Polyline2D(polyline) {}

        /**
         * @brief
         * 
         * @param verts 
         */
        Polygon2D(const PointVec2D& verts):
            Polyline2D(verts) {}

        /**
         * @brief
         * 
         */
        virtual ~Polygon2D() {}

        /**
         * @brief 
         * 
         * @param point 
         * @return bool 
         */
        bool containsPoint(const Point2D& point) const;

        /**
         * @brief 
         * 
         * @param points 
         * @return bool 
         */
        bool containsAnyPoint(const PointVec2D& points) const;

        /**
         * @brief
         * 
         * @return Point2D 
         */
        Point2D getMeanPoint() const;

        /**
         * @brief
         * 
         * @note area can be negative (the sign informs if the polygon is
         * clockwise or not)
         * 
         * @return float 
         */
        float getArea() const;

        /**
         * @brief 
         * 
         * @return bool 
         */
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

        /**
         * @brief
         * 
         * @param tf_mat 
         * @return Polygon2D 
         */
        Polygon2D getTransformedPolygon(const std::vector<float>& tf_mat) const;

        /**
         * @brief
         * 
         * @param tf 
         * @return Polygon2D 
         */
        Polygon2D getTransformedPolygon(const Pose2D& tf) const;

        /**
         * @brief
         * 
         * @param frame 
         * @param red 
         * @param green 
         * @param blue 
         * @param alpha 
         * @param line_width 
         * @param z 
         * @return visualization_msgs::Marker 
         */
        visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.1f,
                float z = 0.0f) const override;

        /**
         * @brief 
         * 
         * @param out 
         * @param polygon 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (std::ostream &out, const Polygon2D& polygon);
};

} // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_POLYGON_2D_H

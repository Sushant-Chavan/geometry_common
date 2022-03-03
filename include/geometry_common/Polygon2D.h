#ifndef KELO_GEOMETRY_COMMON_POLYGON_2D_H
#define KELO_GEOMETRY_COMMON_POLYGON_2D_H

#include <visualization_msgs/Marker.h>

#include <geometry_common/Polyline2D.h>

namespace kelo::geometry_common
{

/**
 * @brief Represents a polygon as a sequence of consecutive 2D waypoints
 * (or vertices). A N-sided polygon is represented by N vertices.An implicit
 * assumption is that there exists a polygon edge from the last to first node
 * in the sequence. \n \n
 * For example, a triangle with vertices [**a**, **b**, **c**] will have the 
 * following edges:
 * 1. **a** -> **b**
 * 2. **b** -> **c**
 * 3. **c** -> **a**
 * 
 */
class Polygon2D : public Polyline2D
{
    public:
        using Ptr = std::shared_ptr<Polygon2D>;
        using ConstPtr = std::shared_ptr<const Polygon2D>;

        /**
         * @brief Construct a new empty Polygon2D object
         * 
         */
        Polygon2D():
            Polyline2D() {}

        /**
         * @brief Copy constructor
         * 
         * @param polygon The polygon to be copied
         */
        Polygon2D(const Polygon2D& polygon):
            Polyline2D(polygon) {}

        /**
         * @brief Construct a new Polygon2D object from polyline
         * 
         * @param polyline A polyline representing the boundary of the polygon.
         * The line must not have the first vertex repeated at the end of the polyline
         */
        Polygon2D(const Polyline2D& polyline):
            Polyline2D(polyline) {}

        /**
         * @brief Construct a new Polygon2D object from a vector of 2D points
         * 
         * @param verts An ordered vector of 2D points representing the vertices
         * of the polygon
         */
        Polygon2D(const PointVec2D& verts):
            Polyline2D(verts) {}

        /**
         * @brief Destroy the Polygon2D object
         * 
         */
        virtual ~Polygon2D() {}

        /**
         * @brief Check if a 2D point lies within the polygon.
         * 
         * @param point The 2D point to be checked
         * @return bool True if the point lies inside the polygon, false otherwise
         */
        bool containsPoint(const Point2D& point) const;

        /**
         * @brief Check if atleast one of the point from the input points vector
         * lies within the polygon
         * 
         * @param points A vector of points to be checked
         * @return bool True if even one of the points lies inside the polygon,
         * false otherwise
         */
        bool containsAnyPoint(const PointVec2D& points) const;

        /**
         * @brief Get the mean of all the polygon vertices
         * 
         * @return Point2D The computed mean point
         */
        Point2D getMeanPoint() const;

        /**
         * @brief Get the area of the polygon
         * 
         * @note If the sign of the area is negative, it means the polygon
         * vertices are specified in a clock-wise winding order
         * 
         * @return float The area of the polygon
         */
        float getArea() const;

        /**
         * @brief Check if the polygon is convex
         * 
         * @return bool True if polygon is convex, false otherwise
         */
        bool isConvex() const;

        /**
         * @brief Find convex hull from the union of 2 polygons.
         * 
         * @param polygon_a First polygon that will be part of the union
         * @param polygon_b Second polygon that will be part of the union
         * @return Polygon2D A polygon representing the convex hull of the 
         * unified input polygons
         */
        static Polygon2D calcConvexHullOfPolygons(
                const Polygon2D& polygon_a,
                const Polygon2D& polygon_b);

        /**
         * @brief Get a transformed copy of the polygon using a 2D transform matrix.
         * 
         * @param tf_mat A 2D homogeneous transform matrix as a 1-dimensional
         * row-major vector
         * @return Polygon2D A transformed copy of this polygon
         */
        Polygon2D getTransformedPolygon(const std::vector<float>& tf_mat) const;

        /**
         * @brief Get a transformed copy of the polygon using a 2D transform object.
         * 
         * @param tf The 2D transformation [x, y, theta] as a Pose2D object
         * @return Polygon2D A transformed copy of this polygon
         */
        Polygon2D getTransformedPolygon(const Pose2D& tf) const;

        /**
         * @brief Get an RViz visualization marker for the polyline object
         * 
         * @param frame The frame in which the polyline marker points are specified
         * @param red The red color-component to be used in the line marker
         * color in the range [0.0, 1.0]
         * @param green The green color-component to be used in the line marker
         * color in the range [0.0, 1.0]
         * @param blue The blue color-component to be used in the line marker
         * color in the range [0.0, 1.0]
         * @param alpha The transparency of the generated line marker
         *  in the range [0.0, 1.0]
         * @param line_width The width of the line marker
         * @param z The Z-coordinate to be appended to all vertices of the line
         * string to represent the 2D point in a 3D space. (Default: 0.0)
         * @return visualization_msgs::Marker A marker object representing the polyline
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
         * @brief Append the polygon information as string to the input stream object
         * 
         * @param out The stream object to which the polygon information should be appended
         * @param polygon The polygon whose data should be appended to the stream object
         * @return std::ostream& The stream object representing the concatenation
         * of the input stream and the polygon information
         */
        friend std::ostream& operator << (std::ostream &out, const Polygon2D& polygon);
};

} // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_POLYGON_2D_H

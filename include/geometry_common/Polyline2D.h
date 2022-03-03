#ifndef KELO_GEOMETRY_COMMON_POLYLINE_2D_H
#define KELO_GEOMETRY_COMMON_POLYLINE_2D_H

#include <visualization_msgs/Marker.h>

#include <geometry_common/Point2D.h>

namespace kelo::geometry_common
{

/**
 * @brief 
 * 
 */
class Polyline2D
{
    public:
        PointVec2D vertices;

        /**
         * @brief
         * 
         */
        Polyline2D() = default;

        /**
         * @brief
         * 
         * @param polyline 
         */
        Polyline2D(const Polyline2D& polyline):
            vertices(polyline.vertices) {}

        /**
         * @brief
         * 
         * @param verts 
         */
        Polyline2D(const PointVec2D& verts):
            vertices(verts) {}

        /**
         * @brief
         * 
         */
        virtual ~Polyline2D() {}

        /**
         * @brief 
         * 
         * @param tf_mat 
         */
        virtual void transform(const std::vector<float>& tf_mat);

        /**
         * @brief 
         * 
         * @param tf 
         */
        virtual void transform(const Pose2D& tf);

        /**
         * @brief
         * 
         * @param tf_mat 
         * @return Polyline2D 
         */
        Polyline2D getTransformedPolyline(const std::vector<float>& tf_mat) const;

        /**
         * @brief
         * 
         * @param tf 
         * @return Polyline2D 
         */
        Polyline2D getTransformedPolyline(const Pose2D& tf) const;

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
        virtual visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.1f,
                float z = 0.0f) const;

        /**
         * @brief 
         * 
         * @return size_t 
         */
        size_t size() const
        {
            return vertices.size();
        }

        /**
         * @brief 
         * 
         * @param index 
         * @return Point2D& 
         */
        Point2D& operator [] (unsigned int index);

        /**
         * @brief 
         * 
         * @param index 
         * @return const Point2D& 
         */
        const Point2D& operator [] (unsigned int index) const;

        /**
         * @brief 
         * 
         * @param out 
         * @param polyline 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (std::ostream &out, const Polyline2D& polyline);
};

} // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_POLYLINE_2D_H

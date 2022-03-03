#ifndef KELO_GEOMETRY_COMMON_POLYLINE_2D_H
#define KELO_GEOMETRY_COMMON_POLYLINE_2D_H

#include <visualization_msgs/Marker.h>

#include <geometry_common/Point2D.h>

namespace kelo::geometry_common
{

class Polyline2D
{
    public:
        PointVec2D vertices;

        Polyline2D() = default;

        Polyline2D(const Polyline2D& polyline):
            vertices(polyline.vertices) {}

        Polyline2D(const PointVec2D& verts):
            vertices(verts) {}

        virtual ~Polyline2D() {}

        virtual void transform(const std::vector<float>& tf_mat);

        virtual void transform(const Pose2D& tf);

        Polyline2D getTransformedPolyline(const std::vector<float>& tf_mat) const;

        Polyline2D getTransformedPolyline(const Pose2D& tf) const;

        virtual visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.1f,
                float z = 0.0f) const;

        friend std::ostream& operator << (std::ostream &out, const Polyline2D& line_segment);
};

} // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_POLYLINE_2D_H

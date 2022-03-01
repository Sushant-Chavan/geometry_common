#ifndef KELO_GEOMETRY_COMMON_POLYGON_H
#define KELO_GEOMETRY_COMMON_POLYGON_H

#include <visualization_msgs/Marker.h>

#include <geometry_common/Point2d.h>

namespace kelo::geometry_common
{

class Polygon2D
{
    public:
        using Points2D = std::vector<Point2D>;

        Points2D corners;

        Polygon2D() = default;

        Polygon2D(const Polygon2D& polygon):
            corners(polygon.corners) {}

        Polygon2D(const Points2D& cornerPts):
            corners(cornerPts) {}

        virtual ~Polygon2D();

        visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.1f,
                double z = 0.0) const;

        friend std::ostream& operator << (std::ostream &out, const Polygon2D& line_segment);
};

} // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_POLYGON_H

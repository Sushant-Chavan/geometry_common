#ifndef KELO_GEOMETRY_COMMON_LINE_SEGMENT_2D_H
#define KELO_GEOMETRY_COMMON_LINE_SEGMENT_2D_H

#include <math.h>
#include <string>
#include <iostream>
#include <visualization_msgs/Marker.h>

#include <geometry_common/Point2D.h>

namespace kelo::geometry_common
{

class LineSegment2D
{
    public:
        Point2D start, end;

        LineSegment2D(float start_x = 0.0f, float start_y = 0.0f,
                      float end_x = 0.0f, float end_y = 0.0f):
            start(start_x, start_y), end(end_x, end_y) {}

        LineSegment2D(const Point2D &start, const Point2D &end):
            start(start), end(end) {}

        LineSegment2D(const LineSegment2D &line_segment):
            LineSegment2D(line_segment.start, line_segment.end) {}

        virtual ~LineSegment2D();

        float getAngle() const;

        float getLength() const;

        float getSlope() const;

        float getConstant() const;

        Point2D getCenter() const;

        Point2D getUnitVector() const;

        bool isIntersecting(const LineSegment2D& line_segment) const;

        bool getIntersectionPoint(
                const LineSegment2D& line_segment,
                Point2D& intersection_point) const;

        Point2D getClosestPointFrom(const Point2D& point) const;

        float getMinDistFrom(const Point2D& point) const;

        bool containsPoint(
                const Point2D& point,
                float dist_threshold = 1e-3f) const;

        visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.1f) const;

        friend std::ostream& operator << (std::ostream &out, const LineSegment2D& line_segment);
};

} // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_LINE_SEGMENT_2D_H

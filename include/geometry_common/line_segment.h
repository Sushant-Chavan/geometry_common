#ifndef KELO_GEOMETRY_COMMON_LINE_SEGMENT_H
#define KELO_GEOMETRY_COMMON_LINE_SEGMENT_H

#include <math.h>
#include <string>
#include <iostream>
#include <visualization_msgs/Marker.h>

#include <geometry_common/point.h>

namespace geometry_common
{

class LineSegment
{
    public:
        Point start, end;

        LineSegment(float start_x = 0.0, float start_y = 0.0,
                    float end_x = 0.0, float end_y = 0.0):
            start(start_x, start_y, 0.0f), end(end_x, end_y, 0.0f) {};

        LineSegment(const Point &start, const Point &end):
            start(start), end(end) {};

        LineSegment(const LineSegment &line_segment):
            start(line_segment.start), end(line_segment.end) {};

        virtual ~LineSegment();

        float getAngle() const;

        float getLength() const;

        float getSlope() const;

        float getConstant() const;

        Point getCenter() const;

        Point getUnitVector() const;

        visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.1f) const;

        friend std::ostream& operator << (std::ostream &out, const LineSegment& line_segment);
};

} // namespace geometry_common
#endif // KELO_GEOMETRY_COMMON_LINE_SEGMENT_H

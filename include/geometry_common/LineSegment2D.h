#ifndef KELO_GEOMETRY_COMMON_LINE_SEGMENT_2D_H
#define KELO_GEOMETRY_COMMON_LINE_SEGMENT_2D_H

#include <math.h>
#include <string>
#include <iostream>
#include <visualization_msgs/Marker.h>

#include <geometry_common/Point2D.h>

namespace kelo::geometry_common
{

/**
 * @brief 
 * 
 */
class LineSegment2D
{
    public:
        Point2D start, end;

        using Ptr = std::shared_ptr<LineSegment2D>;
        using ConstPtr = std::shared_ptr<const LineSegment2D>;

        /**
         * @brief
         * 
         * @param start_x 
         * @param start_y 
         * @param end_x 
         * @param end_y 
         */
        LineSegment2D(float start_x = 0.0f, float start_y = 0.0f,
                      float end_x = 0.0f, float end_y = 0.0f):
            start(start_x, start_y), end(end_x, end_y) {}

        /**
         * @brief
         * 
         * @param start 
         * @param end 
         */
        LineSegment2D(const Point2D &start, const Point2D &end):
            start(start), end(end) {}

        /**
         * @brief
         * 
         * @param line_segment 
         */
        LineSegment2D(const LineSegment2D &line_segment):
            LineSegment2D(line_segment.start, line_segment.end) {}

        /**
         * @brief
         * 
         */
        virtual ~LineSegment2D();

        /**
         * @brief
         * 
         * @return float 
         */
        float getAngle() const;

        /**
         * @brief
         * 
         * @return float 
         */
        float getLength() const;

        /**
         * @brief
         * 
         * @return float 
         */
        float getSlope() const;

        /**
         * @brief
         * 
         * @return float 
         */
        float getConstant() const;

        /**
         * @brief
         * 
         * @return Point2D 
         */
        Point2D getCenter() const;

        /**
         * @brief
         * 
         * @return Point2D 
         */
        Point2D getUnitVector() const;

        /**
         * @brief 
         * 
         * @param line_segment 
         * @return bool 
         */
        bool isIntersecting(const LineSegment2D& line_segment) const;

        /**
         * @brief
         * 
         * @param line_segment 
         * @param intersection_point 
         * @return bool 
         */
        bool getIntersectionPoint(
                const LineSegment2D& line_segment,
                Point2D& intersection_point) const;

        /**
         * @brief
         * 
         * @param point 
         * @return Point2D 
         */
        Point2D getClosestPointFrom(const Point2D& point) const;

        /**
         * @brief 
         * 
         * @param point 
         * @return float 
         */
        float getMinDistFrom(const Point2D& point) const;

        /**
         * @brief 
         * 
         * @param point 
         * @param dist_threshold 
         * @return bool 
         */
        bool containsPoint(
                const Point2D& point,
                float dist_threshold = 1e-3f) const;

        /**
         * @brief
         * 
         * @param frame 
         * @param red 
         * @param green 
         * @param blue 
         * @param alpha 
         * @param line_width 
         * @return visualization_msgs::Marker 
         */
        visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.1f) const;

        /**
         * @brief 
         * 
         * @param out 
         * @param line_segment 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (std::ostream &out, const LineSegment2D& line_segment);
};

} // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_LINE_SEGMENT_2D_H

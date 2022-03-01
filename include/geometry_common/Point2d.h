#ifndef KELO_GEOMETRY_COMMON_POINT_2D_H
#define KELO_GEOMETRY_COMMON_POINT_2D_H

#include <visualization_msgs/Marker.h>

#include <cmath>

namespace kelo::geometry_common
{

class Point2D
{
    public:
        float x{0.0}, y{0.0};

        Point2D(float x = 0.0, float y = 0.0):
            x(x),
            y(y) {}

        Point2D(const Point2D& point):
            x(point.x),
            y(point.y) {}

        virtual ~Point2D() {}

        inline float getCartDist(const Point2D& p) const
        {
            return std::sqrt(getCartDistSquared(p));
        };

        inline float getCartDistSquared(const Point2D& p) const
        {
            return std::pow(x - p.x, 2) + std::pow(y - p.y, 2);
        };

        virtual inline float magnitude() const
        {
            return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        };

        Point2D normalise() const;

        visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float diameter = 0.2f,
                double z = 0.0) const;

        friend Point2D operator - (const Point2D& p1, const Point2D& p2);

        friend Point2D operator + (const Point2D& p1, const Point2D& p2);

        friend Point2D operator * (const Point2D& p1, float scalar);

        friend std::ostream& operator << (std::ostream& out, const Point2D& point);

        friend bool operator == (const Point2D& p1, const Point2D& p2);
};

}; // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_POINT_H

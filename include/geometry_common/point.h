#ifndef GEOMETRY_COMMON_POINT_H
#define GEOMETRY_COMMON_POINT_H

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>

#include <cmath>

namespace geometry_common
{

class Point
{
    public:
        float x, y, z;

        Point(float x = 0.0, float y = 0.0, float z = 0.0):
            x(x),
            y(y),
            z(z) {};

        Point(const Point& point):
            x(point.x),
            y(point.y),
            z(point.z) {};

        Point(const geometry_msgs::PointStamped& point):
            x(point.point.x),
            y(point.point.y),
            z(point.point.z) {};

        Point(const geometry_msgs::Point& point):
            x(point.x),
            y(point.y),
            z(point.z) {};

        Point(const geometry_msgs::Point32& point):
            x(point.x),
            y(point.y),
            z(point.z) {};

        virtual ~Point() {};

        geometry_msgs::Point getPoint() const;

        geometry_msgs::Point32 getPoint32() const;

        geometry_msgs::PointStamped getPointStamped(const std::string& frame="map") const;

        inline float getCartDist(const Point& p) const
        {
            return std::sqrt(getCartDistSquared(p));
        };

        inline float getCartDistSquared(const Point& p) const
        {
            return std::pow(x - p.x, 2) + std::pow(y - p.y, 2) + std::pow(z - p.z, 2);
        };

        inline float magnitude() const
        {
            return std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
        };

        Point normalise() const;

        visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float diameter = 0.2f) const;

        friend Point operator - (const Point& p1, const Point& p2);

        friend Point operator + (const Point& p1, const Point& p2);

        friend Point operator * (const Point& p1, float scalar);

        friend std::ostream& operator << (std::ostream& out, const Point& point);

        friend bool operator == (const Point& p1, const Point& p2);
};

typedef std::vector<Point> PointCloud;

}; // namespace geometry_common
#endif // GEOMETRY_COMMON_POINT_H

#ifndef GEOMETRY_COMMON_POINT_H
#define GEOMETRY_COMMON_POINT_H

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>

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
            return sqrt(getCartDistSquared(p));
        };

        inline float getCartDistSquared(const Point& p) const
        {
            return pow(x - p.x, 2) + pow(y - p.y, 2) + pow(z - p.z, 2);
        };

        inline float magnitude() const
        {
            return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
        };

        Point normalise() const;

        friend Point operator - (const Point& p1, const Point& p2);

        friend Point operator + (const Point& p1, const Point& p2);

        friend Point operator * (const Point& p1, float scalar);

        friend std::ostream& operator << (std::ostream& out, const Point& point);

        friend bool operator == (const Point& p1, const Point& p2);
};

}; // namespace geometry_common
#endif // GEOMETRY_COMMON_POINT_H

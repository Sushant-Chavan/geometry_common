#ifndef KELO_GEOMETRY_COMMON_POINT_3D_H
#define KELO_GEOMETRY_COMMON_POINT_3D_H

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>

#include <geometry_common/Point2D.h>

namespace kelo::geometry_common
{

class Point3D
{
    public:
        float x{0.0f}, y{0.0f}, z{0.0f};

        using Ptr = std::shared_ptr<Point3D>;
        using ConstPtr = std::shared_ptr<const Point3D>;

        Point3D(float _x = 0.0f, float _y = 0.0f, float _z = 0.0f):
            x(_x),
            y(_y),
            z(_z) {}

        Point3D(const Point2D& point, float _z = 0.0f):
            Point3D(point.x, point.y, _z) {}

        Point3D(const Point3D& point):
            Point3D(point.x, point.y, point.z) {}

        Point3D(const geometry_msgs::PointStamped& point):
            Point3D(point.point) {}

        Point3D(const geometry_msgs::Point& point):
            Point3D(point.x, point.y, point.z) {}

        Point3D(const geometry_msgs::Point32& point):
            Point3D(point.x, point.y, point.z) {}

        virtual ~Point3D() {}

        geometry_msgs::Point getPoint() const;

        geometry_msgs::Point32 getPoint32() const;

        geometry_msgs::PointStamped getPointStamped(const std::string& frame = "map") const;

        inline float getCartDist(const Point3D& p) const
        {
            return std::sqrt(getCartDistSquared(p));
        };

        inline float getCartDistSquared(const Point3D& p) const
        {
            return std::pow(x - p.x, 2) + std::pow(y - p.y, 2) + std::pow(z - p.z, 2);
        };

        inline float magnitude() const
        {
            return std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
        };

        Point3D normalise() const;

        void transform(const std::vector<float>& tf_mat);

        Point3D getTransformedPoint(const std::vector<float>& tf_mat) const;

        visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float diameter = 0.2f) const;

        friend Point3D operator - (const Point3D& p1, const Point3D& p2);

        friend Point3D operator + (const Point3D& p1, const Point3D& p2);

        friend Point3D operator * (const Point3D& p1, float scalar);

        friend std::ostream& operator << (std::ostream& out, const Point3D& point);

        friend bool operator == (const Point3D& p1, const Point3D& p2);
};

using Vec3D = Point3D;
using PointVec3D = std::vector<Point3D>;
using PointCloud3D = std::vector<Point3D>;

}; // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_POINT_3D_H

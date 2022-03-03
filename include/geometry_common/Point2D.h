#ifndef KELO_GEOMETRY_COMMON_POINT_2D_H
#define KELO_GEOMETRY_COMMON_POINT_2D_H

#include <visualization_msgs/Marker.h>

#include <cmath>

namespace kelo::geometry_common
{

// Forward declaration 
class Pose2D;

class Point2D
{
    public:
        float x{0.0f}, y{0.0f};

        Point2D(float _x = 0.0f, float _y = 0.0f):
            x(_x),
            y(_y) {}

        Point2D(const Point2D& point):
            Point2D(point.x, point.y) {}

        virtual ~Point2D() {}

        inline float getCartDist(const Point2D& p) const
        {
            return std::sqrt(getCartDistSquared(p));
        };

        inline float getCartDistSquared(const Point2D& p) const
        {
            return std::pow(x - p.x, 2) + std::pow(y - p.y, 2);
        };

        inline float magnitude() const
        {
            return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        };

        Point2D normalise() const;

        float scalarCrossProduct(const Point2D& point) const;

        void transform(const std::vector<float>& tf_mat);

        void transform(const Pose2D& tf);

        Point2D getTransformedPoint(const std::vector<float>& tf_mat) const;

        Point2D getTransformedPoint(const Pose2D& tf) const;

        Point2D getTransformedPoint(
                const std::vector<float>& tf_mat,
                const Point2D& pt) const;

        visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float diameter = 0.2f,
                float z = 0.0f) const;

        friend Point2D operator - (const Point2D& p1, const Point2D& p2);

        friend Point2D operator + (const Point2D& p1, const Point2D& p2);

        friend Point2D operator * (const Point2D& p1, float scalar);

        friend std::ostream& operator << (std::ostream& out, const Point2D& point);

        friend bool operator == (const Point2D& p1, const Point2D& p2);
};

using Vec2D = Point2D;
using PointVec2D = std::vector<Point2D>;
using PointCloud2D = std::vector<Point2D>;

}; // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_POINT_H

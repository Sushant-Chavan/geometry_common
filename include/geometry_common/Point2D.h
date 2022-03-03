#ifndef KELO_GEOMETRY_COMMON_POINT_2D_H
#define KELO_GEOMETRY_COMMON_POINT_2D_H

#include <visualization_msgs/Marker.h>

#include <cmath>

namespace kelo::geometry_common
{

// Forward declaration 
class Pose2D;

/**
 * @brief 
 * 
 */
class Point2D
{
    public:
        float x{0.0f}, y{0.0f};

        using Ptr = std::shared_ptr<Point2D>;
        using ConstPtr = std::shared_ptr<const Point2D>;

        /**
         * @brief
         * 
         * @param _x 
         * @param _y 
         */
        Point2D(float _x = 0.0f, float _y = 0.0f):
            x(_x),
            y(_y) {}

        /**
         * @brief
         * 
         * @param point 
         */
        Point2D(const Point2D& point):
            Point2D(point.x, point.y) {}

        /**
         * @brief
         * 
         */
        virtual ~Point2D() {}

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float getCartDist(const Point2D& p) const
        {
            return std::sqrt(getCartDistSquared(p));
        };

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float getCartDistSquared(const Point2D& p) const
        {
            return std::pow(x - p.x, 2) + std::pow(y - p.y, 2);
        };

        /**
         * @brief 
         * 
         * @return float 
         */
        inline float magnitude() const
        {
            return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        };

        /**
         * @brief 
         * 
         * @return Point2D 
         */
        Point2D normalise() const;


        /**
         * @brief 
         * 
         * @param point 
         * @return float 
         */
        float scalarCrossProduct(const Point2D& point) const;

        /**
         * @brief 
         * 
         * @param tf_mat 
         */
        void transform(const std::vector<float>& tf_mat);

        /**
         * @brief 
         * 
         * @param tf 
         */
        void transform(const Pose2D& tf);

        /**
         * @brief
         * 
         * @param tf_mat 
         * @return Point2D 
         */
        Point2D getTransformedPoint(const std::vector<float>& tf_mat) const;

        /**
         * @brief
         * 
         * @param tf 
         * @return Point2D 
         */
        Point2D getTransformedPoint(const Pose2D& tf) const;

        /**
         * @brief
         * 
         * @param tf_mat 
         * @param pt 
         * @return Point2D 
         */
        Point2D getTransformedPoint(
                const std::vector<float>& tf_mat,
                const Point2D& pt) const;

        /**
         * @brief
         * 
         * @param frame 
         * @param red 
         * @param green 
         * @param blue 
         * @param alpha 
         * @param diameter 
         * @param z 
         * @return visualization_msgs::Marker 
         */
        visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float diameter = 0.2f,
                float z = 0.0f) const;

        /**
         * @brief 
         * 
         * @param p1 
         * @param p2 
         * @return Point2D 
         */
        friend Point2D operator - (const Point2D& p1, const Point2D& p2);

        /**
         * @brief 
         * 
         * @param p1 
         * @param p2 
         * @return Point2D 
         */
        friend Point2D operator + (const Point2D& p1, const Point2D& p2);

        /**
         * @brief 
         * 
         * @param p1 
         * @param scalar 
         * @return Point2D 
         */
        friend Point2D operator * (const Point2D& p1, float scalar);

        /**
         * @brief 
         * 
         * @param out 
         * @param point 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (std::ostream& out, const Point2D& point);

        /**
         * @brief 
         * 
         * @param p1 
         * @param p2 
         * @return bool 
         */
        friend bool operator == (const Point2D& p1, const Point2D& p2);
};

/**
 * @brief 
 * 
 */
using Vec2D = Point2D;

/**
 * @brief 
 * 
 */
using PointVec2D = std::vector<Point2D>;

/**
 * @brief 
 * 
 */
using PointCloud2D = std::vector<Point2D>;

}; // namespace kelo::geometry_common
#endif // KELO_GEOMETRY_COMMON_POINT_H

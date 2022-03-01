#ifndef KELO_GEOMETRY_COMMON_POSE_2D_H
#define KELO_GEOMETRY_COMMON_POSE_2D_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

#include <geometry_common/Point.h>

#include <cmath>

namespace kelo::geometry_common
{

class Pose2d
{
    public:
        float x, y, theta;

        Pose2d(float _x = 0.0f, float _y = 0.0f, float _theta = 0.0f):
            x(_x), y(_y), theta(_theta) {};

        Pose2d(const Pose2d &pose):
            x(pose.x), y(pose.y), theta(pose.theta) {};

        Pose2d(const geometry_msgs::PoseStamped &pose);

        Pose2d(const geometry_msgs::Pose &pose);

        Pose2d(const std::vector<float>& mat);

        Pose2d(const tf::StampedTransform &stamped_transform);

        virtual ~Pose2d();

        geometry_msgs::PoseStamped getPoseStamped(const std::string& frame="map") const;

        geometry_msgs::Pose getPose() const;

        std::vector<float> getMat() const;

        visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float size_x = 0.3f,
                float size_y = 0.05f,
                float size_z = 0.05f) const;

        inline float getCartDist(const Pose2d& p) const
        {
            return std::sqrt(getCartDistSquared(p));
        };

        inline float getCartDistSquared(const Pose2d& p) const
        {
            return std::pow(x - p.x, 2) + std::pow(y - p.y, 2);
        };

        inline float getCartDist(const Point& p) const
        {
            return std::sqrt(pow(x - p.x, 2) + std::pow(y - p.y, 2));
        };

        static float getThetaFromQuaternion(const geometry_msgs::Quaternion& q);

        static float getThetaFromQuaternion(float qx, float qy, float qz, float qw);

        static void getQuaternionFromTheta(float _theta, float& qz, float& qw);

        std::string str() const;

        friend Pose2d operator - (const Pose2d& p1, const Pose2d& p2);

        friend Pose2d operator * (const Pose2d& pose, float scalar);

        friend bool operator == (const Pose2d& p1, const Pose2d& p2);

        friend std::ostream& operator << (std::ostream &out, const Pose2d& pose_2d);
};

} // namespace kelo::geometry_common

#endif // KELO_GEOMETRY_COMMON_POSE_2D_H

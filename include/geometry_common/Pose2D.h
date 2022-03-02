#ifndef KELO_GEOMETRY_COMMON_POSE_2D_H
#define KELO_GEOMETRY_COMMON_POSE_2D_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

#include <geometry_common/Point3D.h>

#include <cmath>

namespace kelo::geometry_common
{

class Pose2D
{
    public:
        float x, y, theta;

        Pose2D(float _x = 0.0f, float _y = 0.0f, float _theta = 0.0f):
            x(_x), y(_y), theta(_theta) {};

        Pose2D(const Pose2D &pose):
            x(pose.x), y(pose.y), theta(pose.theta) {};

        Pose2D(const geometry_msgs::PoseStamped &pose);

        Pose2D(const geometry_msgs::Pose &pose);

        Pose2D(const std::vector<float>& mat);

        Pose2D(const tf::StampedTransform &stamped_transform);

        virtual ~Pose2D();

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

        inline float getCartDist(const Pose2D& p) const
        {
            return std::sqrt(getCartDistSquared(p));
        };

        inline float getCartDistSquared(const Pose2D& p) const
        {
            return std::pow(x - p.x, 2) + std::pow(y - p.y, 2);
        };

        inline float getCartDist(const Point3D& p) const
        {
            return std::sqrt(pow(x - p.x, 2) + std::pow(y - p.y, 2));
        };

        static float getThetaFromQuaternion(const geometry_msgs::Quaternion& q);

        static float getThetaFromQuaternion(float qx, float qy, float qz, float qw);

        static void getQuaternionFromTheta(float _theta, float& qz, float& qw);

        void transform(const std::vector<float>& tf_mat);

        void transform(const Pose2D& tf);

        Pose2D getTransformedPose(const std::vector<float> tf_mat) const;

        Pose2D getTransformedPose(const Pose2D& tf) const;

        std::vector<float> getInverseTransformMat() const;

        Pose2D getInverseTransform() const;

        std::string str() const;

        friend Pose2D operator - (const Pose2D& p1, const Pose2D& p2);

        friend Pose2D operator * (const Pose2D& pose, float scalar);

        friend bool operator == (const Pose2D& p1, const Pose2D& p2);

        friend std::ostream& operator << (std::ostream &out, const Pose2D& pose_2d);
};

} // namespace kelo::geometry_common

#endif // KELO_GEOMETRY_COMMON_POSE_2D_H
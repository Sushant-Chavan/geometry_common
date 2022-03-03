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

/**
 * @brief 
 * 
 */
class Pose2D
{
    public:
        float x, y, theta;

        using Ptr = std::shared_ptr<Pose2D>;
        using ConstPtr = std::shared_ptr<const Pose2D>;

        /**
         * @brief
         * 
         * @param _x 
         * @param _y 
         * @param _theta 
         */
        Pose2D(float _x = 0.0f, float _y = 0.0f, float _theta = 0.0f):
            x(_x), y(_y), theta(_theta) {};

        /**
         * @brief
         * 
         * @param pose 
         */
        Pose2D(const Pose2D &pose):
            x(pose.x), y(pose.y), theta(pose.theta) {};

        /**
         * @brief
         * 
         * @param pose 
         */
        Pose2D(const geometry_msgs::PoseStamped &pose);

        /**
         * @brief
         * 
         * @param pose 
         */
        Pose2D(const geometry_msgs::Pose &pose);

        /**
         * @brief
         * 
         * @param mat 
         */
        Pose2D(const std::vector<float>& mat);

        /**
         * @brief
         * 
         * @param stamped_transform 
         */
        Pose2D(const tf::StampedTransform &stamped_transform);

        /**
         * @brief
         * 
         */
        virtual ~Pose2D();

        /**
         * @brief
         * 
         * @param frame 
         * @return geometry_msgs::PoseStamped 
         */
        geometry_msgs::PoseStamped getPoseStamped(const std::string& frame="map") const;

        /**
         * @brief
         * 
         * @return geometry_msgs::Pose 
         */
        geometry_msgs::Pose getPose() const;

        /**
         * @brief
         * 
         * @return std::vector<float> 
         */
        std::vector<float> getMat() const;

        /**
         * @brief
         * 
         * @param frame 
         * @param red 
         * @param green 
         * @param blue 
         * @param alpha 
         * @param size_x 
         * @param size_y 
         * @param size_z 
         * @return visualization_msgs::Marker 
         */
        visualization_msgs::Marker getMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float size_x = 0.3f,
                float size_y = 0.05f,
                float size_z = 0.05f) const;

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float getCartDist(const Pose2D& p) const
        {
            return std::sqrt(getCartDistSquared(p));
        };

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float getCartDistSquared(const Pose2D& p) const
        {
            return std::pow(x - p.x, 2) + std::pow(y - p.y, 2);
        };

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float getCartDist(const Point3D& p) const
        {
            return std::sqrt(pow(x - p.x, 2) + std::pow(y - p.y, 2));
        };

        /**
         * @brief
         * 
         * @param q 
         * @return float 
         */
        static float getThetaFromQuaternion(const geometry_msgs::Quaternion& q);

        /**
         * @brief
         * 
         * @param qx 
         * @param qy 
         * @param qz 
         * @param qw 
         * @return float 
         */
        static float getThetaFromQuaternion(float qx, float qy, float qz, float qw);

        /**
         * @brief
         * 
         * @param _theta 
         * @param qz 
         * @param qw 
         */
        static void getQuaternionFromTheta(float _theta, float& qz, float& qw);

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
         * @return Pose2D 
         */
        Pose2D getTransformedPose(const std::vector<float> tf_mat) const;

        /**
         * @brief
         * 
         * @param tf 
         * @return Pose2D 
         */
        Pose2D getTransformedPose(const Pose2D& tf) const;

        /**
         * @brief
         * 
         * @return std::vector<float> 
         */
        std::vector<float> getInverseTransformMat() const;

        /**
         * @brief
         * 
         * @return Pose2D 
         */
        Pose2D getInverseTransform() const;

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string str() const;

        /**
         * @brief 
         * 
         * @param p1 
         * @param p2 
         * @return Pose2D 
         */
        friend Pose2D operator - (const Pose2D& p1, const Pose2D& p2);

        /**
         * @brief 
         * 
         * @param pose 
         * @param scalar 
         * @return Pose2D 
         */
        friend Pose2D operator * (const Pose2D& pose, float scalar);

        /**
         * @brief 
         * 
         * @param p1 
         * @param p2 
         * @return bool 
         */
        friend bool operator == (const Pose2D& p1, const Pose2D& p2);

        /**
         * @brief 
         * 
         * @param out 
         * @param pose_2d 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (std::ostream &out, const Pose2D& pose_2d);
};

using Velocity2D = Pose2D;
using Acceleration2D = Pose2D;

} // namespace kelo::geometry_common

#endif // KELO_GEOMETRY_COMMON_POSE_2D_H

#include <geometry_common/pose_2d.h>
#include <geometry_common/utils.h>
#include <cmath>

namespace geometry_common
{

Pose2d::Pose2d(const geometry_msgs::PoseStamped &pose)
{
    x = pose.pose.position.x;
    y = pose.pose.position.y;
    theta = Pose2d::getThetaFromQuaternion(pose.pose.orientation);
}

Pose2d::Pose2d(const geometry_msgs::Pose &pose)
{
    x = pose.position.x;
    y = pose.position.y;
    theta = Pose2d::getThetaFromQuaternion(pose.orientation);
}

Pose2d::Pose2d(const std::vector<float>& mat)
{
    assert(mat.size() == 9);
    x = mat[2];
    y = mat[5];
    theta = std::atan2(mat[3], mat[0]);
}

Pose2d::Pose2d(const tf::StampedTransform &stamped_transform)
{
    x = stamped_transform.getOrigin().x();
    y = stamped_transform.getOrigin().y();

    tf::Quaternion quat = stamped_transform.getRotation();
    theta = tf::getYaw(quat);
}

Pose2d::~Pose2d()
{
}

geometry_msgs::PoseStamped Pose2d::getPoseStamped(const std::string& frame) const
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.pose = getPose();
    return pose;
}

geometry_msgs::Pose Pose2d::getPose() const
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0f;
    pose.orientation.x = 0.0f;
    pose.orientation.y = 0.0f;
    float z, w;
    Pose2d::getQuaternionFromTheta(theta, z, w);
    pose.orientation.z = z;
    pose.orientation.w = w;
    return pose;
}

std::vector<float> Pose2d::getMat() const
{
    return Utils::get2DTransformMat(x, y, theta);
}

visualization_msgs::Marker Pose2d::getMarker(const std::string& frame,
        float red, float green, float blue, float alpha,
        float size_x, float size_y, float size_z) const
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::ARROW;
    // marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = size_x;
    marker.scale.y = size_y;
    marker.scale.z = size_z;
    marker.pose = getPose();
    return marker;
}

float Pose2d::getThetaFromQuaternion(const geometry_msgs::Quaternion& q)
{
    return Pose2d::getThetaFromQuaternion(q.x, q.y, q.z, q.w);
}

float Pose2d::getThetaFromQuaternion(float qx, float qy, float qz, float qw)
{
    /* source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code_2 */
    float sinyaw_cospitch = 2 * (qw * qz + qx * qy);
    float cosyaw_cospitch = 1 - 2 * (qy * qy + qz * qz);
    return std::atan2(sinyaw_cospitch, cosyaw_cospitch);
}

void Pose2d::getQuaternionFromTheta(float _theta, float& qz, float& qw)
{
    /* source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code */
    qw = std::cos(_theta * 0.5f);
    qz = std::sin(_theta * 0.5f);
}

std::string Pose2d::str() const
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "<x: " << x
        << ", y: " << y
        << ", theta: " << theta
        << ">";
    return ss.str();
}

Pose2d operator - (const Pose2d& pose_1, const Pose2d& pose_2)
{
    Pose2d diff;
    diff.x = pose_1.x - pose_2.x;
    diff.y = pose_1.y - pose_2.y;
    diff.theta = Utils::getShortestAngle(pose_1.theta, pose_2.theta);
    return diff;
}

Pose2d operator * (const Pose2d& pose, float scalar)
{
    Pose2d scaled;
    scaled.x = pose.x * scalar;
    scaled.y = pose.y * scalar;
    scaled.theta = pose.theta * scalar;
    return scaled;
}

bool operator == (const Pose2d& pose_1, const Pose2d& pose_2)
{
    return ( pose_1.getCartDist(pose_2) < 1e-3f &&
             Utils::getShortestAngle(pose_1.theta, pose_2.theta) < 1e-2f );
}

std::ostream& operator << (std::ostream &out, const Pose2d& pose_2d)
{
    out << "<x: " << pose_2d.x
        << ", y: " << pose_2d.y
        << ", theta: " << pose_2d.theta
        << ">";
    return out;
}

} // namespace geometry_common

#include <gtest/gtest.h>

#include <vector>

#include <geometry_common/Utils.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Utils;
using kelo::geometry_common::WindingOrder;

TEST(UtilsTest, clipAngle)
{
    EXPECT_NEAR(Utils::clipAngle(0.0f), 0.0f, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(M_PI/4), M_PI/4, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(-M_PI/4), -M_PI/4, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(M_PI/2), M_PI/2, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(-M_PI/2), -M_PI/2, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(3.0f), 3.0f, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(-3.0f), -3.0f, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(1.5f*M_PI), -M_PI/2, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(-1.5f*M_PI), M_PI/2, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(2.0f*M_PI), 0.0f, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(-2.0f*M_PI), 0.0f, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(3.1f*M_PI), -0.9f*M_PI, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(-3.1f*M_PI), 0.9f*M_PI, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(4.2f*M_PI), 0.2f*M_PI, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(-4.2f*M_PI), -0.2f*M_PI, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(5.3f*M_PI), -0.7f*M_PI, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(-5.3f*M_PI), 0.7f*M_PI, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(7.75f*M_PI), -M_PI/4, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(-7.75f*M_PI), M_PI/4, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(10.0f*M_PI), 0.0f, 1e-3f);
    EXPECT_NEAR(Utils::clipAngle(-10.0f*M_PI), 0.0f, 1e-3f);
}

TEST(UtilsTest, calcAngleBetweenPoints)
{
    Point2D a(0.0f, 0.0f);
    Point2D b(1.0f, 0.0f);
    Point2D c(2.0f, 1.0f);
    Point2D d(0.0f, 1.0f);
    Point2D e(2.0f, -1.0f);
    EXPECT_NEAR(Utils::calcAngleBetweenPoints(a, b, c), -3*M_PI/4, 1e-3f);
    EXPECT_NEAR(Utils::calcAngleBetweenPoints(c, b, a), 3*M_PI/4, 1e-3f);
    EXPECT_NEAR(Utils::calcAngleBetweenPoints(d, b, a), M_PI/4, 1e-3f);
    EXPECT_NEAR(Utils::calcAngleBetweenPoints(e, b, a), -3*M_PI/4, 1e-3f);
}

TEST(UtilsTest, calcWindingOrder)
{
    Point2D a(0.0f, 0.0f);
    Point2D b(1.0f, 0.0f);
    Point2D c(2.0f, 1.0f);
    Point2D d(2.0f, 0.0f);
    EXPECT_EQ(Utils::calcWindingOrder(a, b, c), WindingOrder::COUNTER_CLOCKWISE);
    EXPECT_EQ(Utils::calcWindingOrder(c, b, a), WindingOrder::CLOCKWISE);
    EXPECT_EQ(Utils::calcWindingOrder(a, b, d), WindingOrder::COLLINEAR);
    EXPECT_EQ(Utils::calcWindingOrder(d, b, a), WindingOrder::COLLINEAR);
    EXPECT_EQ(Utils::calcWindingOrder(a, d, b), WindingOrder::COLLINEAR);
}

TEST(UtilsTest, convertQuaternionToEuler)
{
    float roll, pitch, yaw;
    geometry_msgs::Quaternion q;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;
    q.w = 1.0f;
    Utils::convertQuaternionToEuler(q.x, q.y, q.z, q.w, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, 1e-3f);
    EXPECT_NEAR(pitch, 0.0f, 1e-3f);
    EXPECT_NEAR(yaw, 0.0f, 1e-3f);

    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.4794f;
    q.w = 0.8776f;
    Utils::convertQuaternionToEuler(q.x, q.y, q.z, q.w, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, 1e-3f);
    EXPECT_NEAR(pitch, 0.0f, 1e-3f);
    EXPECT_NEAR(yaw, 1.0f, 1e-3f);

    q.x = 0.0f;
    q.y = 0.0f;
    q.z = -0.4794f;
    q.w = 0.8776f;
    Utils::convertQuaternionToEuler(q.x, q.y, q.z, q.w, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, 1e-3f);
    EXPECT_NEAR(pitch, 0.0f, 1e-3f);
    EXPECT_NEAR(yaw, -1.0f, 1e-3f);

    q.x = 0.1675f;
    q.y = 0.5709f;
    q.z = 0.1675f;
    q.w = 0.7861f;
    Utils::convertQuaternionToEuler(q.x, q.y, q.z, q.w, roll, pitch, yaw);
    EXPECT_NEAR(roll, 1.0f, 1e-3f);
    EXPECT_NEAR(pitch, 1.0f, 1e-3f);
    EXPECT_NEAR(yaw, 1.0f, 1e-3f);

    q.x = 0.0706f;
    q.y = 0.7036f;
    q.z = -0.0706;
    q.w = 0.7036f;
    Utils::convertQuaternionToEuler(q.x, q.y, q.z, q.w, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, 1e-3f);
    EXPECT_NEAR(pitch, M_PI/2, 1e-3f);
    EXPECT_NEAR(yaw, 0.2f, 1e-3f);
    // actual rpy was 0.2, pi/2 and 0.4 but it is also equivalent to 0, pi/2 and 0.2

    q.x = -0.197671f;
    q.y =  0.975170f;
    q.z = -0.097843f;
    q.w =  0.019833f;
    Utils::convertQuaternionToEuler(q.x, q.y, q.z, q.w, roll, pitch, yaw);
    EXPECT_NEAR(roll, -2.9416, 1e-3f);
    EXPECT_NEAR(pitch, 0.0f, 1e-3f);
    EXPECT_NEAR(yaw, -2.7416f, 1e-3f);
    // actual rpy was 0.2, pi and 0.4 but it is also equivalent to (-pi+0.2), 0 and (-pi+0.4)
}

TEST(UtilsTest, convertEulerToQuaternion)
{
    float qx, qy, qz, qw;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    Utils::convertEulerToQuaternion(roll, pitch, yaw, qx, qy, qz, qw);
    EXPECT_NEAR(qx, 0.0f, 1e-3f);
    EXPECT_NEAR(qy, 0.0f, 1e-3f);
    EXPECT_NEAR(qz, 0.0f, 1e-3f);
    EXPECT_NEAR(qw, 1.0f, 1e-3f);

    yaw = 1.0f;
    Utils::convertEulerToQuaternion(roll, pitch, yaw, qx, qy, qz, qw);
    EXPECT_NEAR(qx, 0.0f, 1e-3f);
    EXPECT_NEAR(qy, 0.0f, 1e-3f);
    EXPECT_NEAR(qz, 0.4794f, 1e-3f);
    EXPECT_NEAR(qw, 0.8776f, 1e-3f);

    yaw = -1.0f;
    Utils::convertEulerToQuaternion(roll, pitch, yaw, qx, qy, qz, qw);
    EXPECT_NEAR(qx, 0.0f, 1e-3f);
    EXPECT_NEAR(qy, 0.0f, 1e-3f);
    EXPECT_NEAR(qz, -0.4794f, 1e-3f);
    EXPECT_NEAR(qw, 0.8776f, 1e-3f);

    roll = 1.0f;
    pitch = 1.0f;
    yaw = 1.0f;
    Utils::convertEulerToQuaternion(roll, pitch, yaw, qx, qy, qz, qw);
    EXPECT_NEAR(qx, 0.1675f, 1e-3f);
    EXPECT_NEAR(qy, 0.5709f, 1e-3f);
    EXPECT_NEAR(qz, 0.1675f, 1e-3f);
    EXPECT_NEAR(qw, 0.7861f, 1e-3f);
}

#include <gtest/gtest.h>

#include <vector>

#include <geometry_common/Utils.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Utils;

TEST(UtilsTest, getAngleBetweenPoints)
{
    Point2D a(0.0f, 0.0f);
    Point2D b(1.0f, 0.0f);
    Point2D c(2.0f, 1.0f);
    Point2D d(0.0f, 1.0f);
    Point2D e(2.0f, -1.0f);
    EXPECT_FLOAT_EQ(Utils::getAngleBetweenPoints(a, b, c), -3*M_PI/4)
        << "Angle should be -3pi/4.";
    EXPECT_FLOAT_EQ(Utils::getAngleBetweenPoints(d, b, a), M_PI/4)
        << "Angle should be pi/4.";
    EXPECT_FLOAT_EQ(Utils::getAngleBetweenPoints(e, b, a), -3*M_PI/4)
        << "Angle should be -3pi/4.";
}

TEST(UtilsTest, getEulerFromQuaternion)
{
    float roll, pitch, yaw;
    geometry_msgs::Quaternion q;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;
    q.w = 1.0f;
    Utils::getEulerFromQuaternion(q.x, q.y, q.z, q.w, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, 1e-3f);
    EXPECT_NEAR(pitch, 0.0f, 1e-3f);
    EXPECT_NEAR(yaw, 0.0f, 1e-3f);

    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.4794f;
    q.w = 0.8776f;
    Utils::getEulerFromQuaternion(q.x, q.y, q.z, q.w, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, 1e-3f);
    EXPECT_NEAR(pitch, 0.0f, 1e-3f);
    EXPECT_NEAR(yaw, 1.0f, 1e-3f);

    q.x = 0.0f;
    q.y = 0.0f;
    q.z = -0.4794f;
    q.w = 0.8776f;
    Utils::getEulerFromQuaternion(q.x, q.y, q.z, q.w, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0f, 1e-3f);
    EXPECT_NEAR(pitch, 0.0f, 1e-3f);
    EXPECT_NEAR(yaw, -1.0f, 1e-3f);

    q.x = 0.1675f;
    q.y = 0.5709f;
    q.z = 0.1675f;
    q.w = 0.7861f;
    Utils::getEulerFromQuaternion(q.x, q.y, q.z, q.w, roll, pitch, yaw);
    EXPECT_NEAR(roll, 1.0f, 1e-3f);
    EXPECT_NEAR(pitch, 1.0f, 1e-3f);
    EXPECT_NEAR(yaw, 1.0f, 1e-3f);
}

TEST(UtilsTest, getQuaternionFromEuler)
{
    float qx, qy, qz, qw;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    Utils::getQuaternionFromEuler(roll, pitch, yaw, qx, qy, qz, qw);
    EXPECT_NEAR(qx, 0.0f, 1e-3f);
    EXPECT_NEAR(qy, 0.0f, 1e-3f);
    EXPECT_NEAR(qz, 0.0f, 1e-3f);
    EXPECT_NEAR(qw, 1.0f, 1e-3f);

    yaw = 1.0f;
    Utils::getQuaternionFromEuler(roll, pitch, yaw, qx, qy, qz, qw);
    EXPECT_NEAR(qx, 0.0f, 1e-3f);
    EXPECT_NEAR(qy, 0.0f, 1e-3f);
    EXPECT_NEAR(qz, 0.4794f, 1e-3f);
    EXPECT_NEAR(qw, 0.8776f, 1e-3f);

    yaw = -1.0f;
    Utils::getQuaternionFromEuler(roll, pitch, yaw, qx, qy, qz, qw);
    EXPECT_NEAR(qx, 0.0f, 1e-3f);
    EXPECT_NEAR(qy, 0.0f, 1e-3f);
    EXPECT_NEAR(qz, -0.4794f, 1e-3f);
    EXPECT_NEAR(qw, 0.8776f, 1e-3f);

    roll = 1.0f;
    pitch = 1.0f;
    yaw = 1.0f;
    Utils::getQuaternionFromEuler(roll, pitch, yaw, qx, qy, qz, qw);
    EXPECT_NEAR(qx, 0.1675f, 1e-3f);
    EXPECT_NEAR(qy, 0.5709f, 1e-3f);
    EXPECT_NEAR(qz, 0.1675f, 1e-3f);
    EXPECT_NEAR(qw, 0.7861f, 1e-3f);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#include <gtest/gtest.h>
#include <geometry_common/pose_2d.h>
#include <geometry_msgs/Quaternion.h>

TEST(Pose2dTest, constructor)
{
    geometry_common::Pose2d pose1;
    EXPECT_FLOAT_EQ(pose1.x, 0.0f) << "pose1.x is not zero";
    EXPECT_FLOAT_EQ(pose1.y, 0.0f) << "pose1.y is not zero";
    EXPECT_FLOAT_EQ(pose1.theta, 0.0f) << "pose1.theta is not zero";

    geometry_common::Pose2d pose2(1.0f, 2.0f, 1.5f);
    EXPECT_FLOAT_EQ(pose2.x, 1.0f) << "pose2.x is not 1.0f";
    EXPECT_FLOAT_EQ(pose2.y, 2.0f) << "pose2.y is not 2.0f";
    EXPECT_FLOAT_EQ(pose2.theta, 1.5f) << "pose2.theta is not 1.5f";
}

TEST(Pose2dTest, getThetaFromQuaternion)
{
    geometry_msgs::Quaternion q;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;
    q.w = 1.0f;
    float theta1 = geometry_common::Pose2d::getThetaFromQuaternion(q);
    EXPECT_NEAR(theta1, 0.0f, 1e-3f);

    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.4794f;
    q.w = 0.8776f;
    float theta2 = geometry_common::Pose2d::getThetaFromQuaternion(q);
    EXPECT_NEAR(theta2, 1.0f, 1e-3f);

    q.x = 0.0f;
    q.y = 0.0f;
    q.z = -0.4794f;
    q.w = 0.8776f;
    float theta3 = geometry_common::Pose2d::getThetaFromQuaternion(q);
    EXPECT_NEAR(theta3, -1.0f, 1e-3f);

    q.x = 0.1675f;
    q.y = 0.5709f;
    q.z = 0.1675f;
    q.w = 0.7861f;
    float theta4 = geometry_common::Pose2d::getThetaFromQuaternion(q);
    EXPECT_NEAR(theta4, 1.0f, 1e-3f);
}

TEST(Pose2dTest, getQuaternionFromTheta)
{
    float qz = 0.0f, qw = 0.0f;

    float theta = 0.0f;
    geometry_common::Pose2d::getQuaternionFromTheta(theta, qz, qw);
    EXPECT_NEAR(qz, 0.0f, 1e-3f);
    EXPECT_NEAR(qw, 1.0f, 1e-3f);

    theta = 1.0f;
    geometry_common::Pose2d::getQuaternionFromTheta(theta, qz, qw);
    EXPECT_NEAR(qz, 0.4794f, 1e-3f);
    EXPECT_NEAR(qw, 0.8776f, 1e-3f);

    theta = -1.0f;
    geometry_common::Pose2d::getQuaternionFromTheta(theta, qz, qw);
    EXPECT_NEAR(qz, -0.4794f, 1e-3f);
    EXPECT_NEAR(qw, 0.8776f, 1e-3f);

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

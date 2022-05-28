#include <gtest/gtest.h>
#include <geometry_common/TransformMatrix3D.h>

using kelo::geometry_common::TransformMatrix3D;

TEST(TransformMatrix3D, multiplication)
{
    TransformMatrix3D tf1(1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f);
    TransformMatrix3D tf2; // identity transformation
    TransformMatrix3D tf3 = tf1 * tf2;
    EXPECT_NEAR(tf3.x(), 1.0f, 1e-3f);
    EXPECT_NEAR(tf3.y(), 2.0f, 1e-3f);
    EXPECT_NEAR(tf3.z(), 3.0f, 1e-3f);
    EXPECT_NEAR(tf3.roll(), 0.1f, 1e-3f);
    EXPECT_NEAR(tf3.pitch(), 0.2f, 1e-3f);
    EXPECT_NEAR(tf3.yaw(), 0.3f, 1e-3f);

}

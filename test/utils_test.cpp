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

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

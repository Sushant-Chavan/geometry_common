#include <gtest/gtest.h>
#include <geometry_common/point.h>

TEST(PointTest, constructor)
{
    geometry_common::Point p;
    EXPECT_EQ(p.x, 0.0f) << "p.x is not zero";
    EXPECT_EQ(p.y, 0.0f) << "p.x is not zero";
    EXPECT_EQ(p.z, 0.0f) << "p.x is not zero";

    geometry_common::Point pt(1.0f, 2.0f, 3.0f);
    EXPECT_EQ(pt.x, 1.0f) << "pt.x is not 1.0f";
    EXPECT_EQ(pt.y, 2.0f) << "pt.x is not 2.0f";
    EXPECT_EQ(pt.z, 3.0f) << "pt.x is not 3.0f";
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

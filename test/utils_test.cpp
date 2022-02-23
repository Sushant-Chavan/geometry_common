#include <gtest/gtest.h>

#include <vector>

#include <geometry_common/point.h>
#include <geometry_common/utils.h>

// Demonstrate some basic assertions.
TEST(UtilsTest, isPolygonConvex) {
    std::vector<geometry_common::Point> convex_polygon(
    {
        geometry_common::Point(0.0f, 0.0f),
        geometry_common::Point(5.0f, 0.0f),
        geometry_common::Point(4.0f, 4.0f),
        geometry_common::Point(0.0f, 3.0f)
    });
    EXPECT_EQ(geometry_common::Utils::isPolygonConvex(convex_polygon), true)
        << "Convex polygon is not found to be convex.";

    std::vector<geometry_common::Point> concave_polygon(
    {
        geometry_common::Point(0.0f, 0.0f),
        geometry_common::Point(5.0f, 0.0f),
        geometry_common::Point(1.0f, 1.0f),
        geometry_common::Point(0.0f, 5.0f)
    });
    EXPECT_EQ(geometry_common::Utils::isPolygonConvex(concave_polygon), false)
        << "Concave polygon is not found to be concave.";
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

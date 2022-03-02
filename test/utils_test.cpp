#include <gtest/gtest.h>

#include <vector>

#include <geometry_common/Point3D.h>
#include <geometry_common/Utils.h>


using kelo::geometry_common::Point2D;
using kelo::geometry_common::Polygon2D;

TEST(UtilsTest, getAngleBetweenPoints)
{
    Point2D a(0.0f, 0.0f);
    Point2D b(1.0f, 0.0f);
    Point2D c(2.0f, 1.0f);
    Point2D d(0.0f, 1.0f);
    Point2D e(2.0f, -1.0f);
    EXPECT_FLOAT_EQ(kelo::geometry_common::Utils::getAngleBetweenPoints(a, b, c), -3*M_PI/4)
        << "Angle should be -3pi/4.";
    EXPECT_FLOAT_EQ(kelo::geometry_common::Utils::getAngleBetweenPoints(d, b, a), M_PI/4)
        << "Angle should be pi/4.";
    EXPECT_FLOAT_EQ(kelo::geometry_common::Utils::getAngleBetweenPoints(e, b, a), -3*M_PI/4)
        << "Angle should be -3pi/4.";
}

TEST(UtilsTest, isPolygonConvex)
{
    Polygon2D convex_polygon(
    {
        Point2D(0.0f, 0.0f),
        Point2D(5.0f, 0.0f),
        Point2D(4.0f, 4.0f),
        Point2D(0.0f, 3.0f)
    });
    EXPECT_EQ(convex_polygon.isConvex(), true)
        << "Convex polygon is not found to be convex.";

    Polygon2D concave_polygon(
    {
        Point2D(0.0f, 0.0f),
        Point2D(5.0f, 0.0f),
        Point2D(1.0f, 1.0f),
        Point2D(0.0f, 5.0f)
    });
    EXPECT_EQ(concave_polygon.isConvex(), false)
        << "Concave polygon is not found to be concave.";

    Polygon2D star_polygon(
    {
        Point2D(0.0f, 0.0f),
        Point2D(2.0f, 5.0f),
        Point2D(4.0f, 0.0f),
        Point2D(0.0f, 3.0f),
        Point2D(4.0f, 3.0f),
    });
    EXPECT_EQ(star_polygon.isConvex(), false)
        << "Star polygon is not found to be concave.";
}

TEST(UtilsTest, calcConvexHullOfPolygons)
{
    Polygon2D polygon_a(
    {
        Point2D(0.0f, 0.0f),
        Point2D(5.0f, 0.0f),
        Point2D(5.0f, 4.0f),
        Point2D(0.0f, 4.0f)
    });
    Polygon2D polygon_b(
    {
        Point2D(3.0f, 2.0f),
        Point2D(9.0f, 1.0f),
        Point2D(9.0f, 3.0f)
    });
    Polygon2D convex_hull = kelo::geometry_common::Utils::calcConvexHullOfPolygons(
            polygon_a, polygon_b);

    EXPECT_EQ(convex_hull.vertices.size(), 6u);
    EXPECT_EQ(convex_hull.vertices[0], polygon_a.vertices[0]);
    EXPECT_EQ(convex_hull.vertices[1], polygon_a.vertices[1]);
    EXPECT_EQ(convex_hull.vertices[2], polygon_b.vertices[1]);
    EXPECT_EQ(convex_hull.vertices[3], polygon_b.vertices[2]);
    EXPECT_EQ(convex_hull.vertices[4], polygon_a.vertices[2]);
    EXPECT_EQ(convex_hull.vertices[5], polygon_a.vertices[3]);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#include <gtest/gtest.h>
#include <geometry_common/LineSegment2D.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::LineSegment2D;

TEST(LineSegment2D, getIntersectionPoint)
{
    LineSegment2D l1(0, 0, 10, 0);
    LineSegment2D l2(5, -5, 5, 5); // perpendicular intersection to l1
    LineSegment2D l3(0, 0, 5, 5); // diagonal with l1 intersecting at start points
    LineSegment2D l4(15, 0, 20, 0); // collinear non-intersecting to l1
    LineSegment2D l5(5, 0, 15, 0); // collinear partially overlapping to l1
    LineSegment2D l6(0, 1, 5, 6); // parallel to l3
    
    Point2D intersection_pt;

    EXPECT_EQ(l1.getIntersectionPoint(l2, intersection_pt), true)
        << "l1 and l2 don't intersect.";
    EXPECT_EQ(intersection_pt == Point2D(5, 0), true)
        << "l1 and l2 don't intersect at (5, 0).";

    EXPECT_EQ(l1.getIntersectionPoint(l3, intersection_pt), true)
        << "l1 and l3 don't intersect.";
    EXPECT_EQ(intersection_pt == Point2D(0, 0), true)
        << "l1 and l3 don't intersect at (0, 0).";

    EXPECT_EQ(l1.getIntersectionPoint(l4, intersection_pt), false)
        << "l1 and l4 intersect.";

    // See Issue #13
    // EXPECT_EQ(l1.getIntersectionPoint(l5, intersection_pt), true)
    //     << "l1 and l5 don't intersect.";

    EXPECT_EQ(l3.getIntersectionPoint(l6, intersection_pt), false)
        << "l3 and l6 intersect.";
}

TEST(LineSegment2D, getClosestPointFrom)
{
    LineSegment2D l(0, 0, 5, 5);
    Point2D p1(0, 0); // start point of l
    Point2D p2(0, 5);
    Point2D p3(10, 10); // collinear point which is not on segment
    Point2D p4(1, 1); // collinear point on line segment

    EXPECT_EQ(l.getClosestPointFrom(p1) == p1, true);

    EXPECT_EQ(l.getClosestPointFrom(p2) == Point2D(2.5f, 2.5f), true);

    EXPECT_EQ(l.getClosestPointFrom(p3) == Point2D(5, 5), true);

    EXPECT_EQ(l.getClosestPointFrom(p4) == p4, true);
}

TEST(LineSegment2D, getMinDistFrom)
{
    LineSegment2D l(0, 0, 5, 5);
    Point2D p1(0, 0); // start point of l
    Point2D p2(0, 5);
    Point2D p3(10, 10); // collinear point which is not on segment
    Point2D p4(1, 1); // collinear point on line segment

    EXPECT_NEAR(l.getMinDistFrom(p1), 0.0f, 1e-3f);

    EXPECT_NEAR(l.getMinDistFrom(p2), 3.535f, 1e-3f);

    EXPECT_NEAR(l.getMinDistFrom(p3), 7.071f, 1e-3f);

    EXPECT_NEAR(l.getMinDistFrom(p4), 0.0f, 1e-3f);
}

TEST(LineSegment2D, containsPoint)
{
    LineSegment2D l(0, 0, 5, 5);
    Point2D p1(0, 0); // start point of l
    Point2D p2(0, 5);
    Point2D p3(10, 10); // collinear point which is not on segment
    Point2D p4(1, 1); // collinear point on line segment

    EXPECT_EQ(l.containsPoint(p1), true);

    EXPECT_EQ(l.containsPoint(p2), false);

    EXPECT_EQ(l.containsPoint(p3), false);

    EXPECT_EQ(l.containsPoint(p4), true);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#ifndef GEOMETRY_COMMON_UTILS_H
#define GEOMETRY_COMMON_UTILS_H

#include <vector>
#include <string>

#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_common/pose_2d.h>
#include <geometry_common/point.h>
#include <geometry_common/line_segment.h>

namespace geometry_common
{

class Utils
{
    public:

        static float roundFloat(
                float value,
                unsigned decimal_places);

        static Point getMeanPoint(
                const PointCloud& points,
                unsigned start_index,
                unsigned end_index);

        static Point getMeanPoint(
                const PointCloud& points);

        template <typename T>
        static Pose2d getMeanPose(
                const T& poses);

        static Point getClosestPoint(
                const PointCloud& points,
                float x=0.0f,
                float y=0.0f,
                float z=0.0f);

        static std::vector<std::vector<Point> > clusterPoints(
                const PointCloud& points,
                float cluster_distance_threshold=0.1f,
                size_t min_cluster_size=3);

        static std::vector<std::vector<Point> > clusterOrderedPoints(
                const PointCloud& points,
                float cluster_distance_threshold=0.1f,
                size_t min_cluster_size=3);

        /* 
         * Source: https://stackoverflow.com/a/2922778/10460994
         */
        static bool isPointInPolygon(
                const std::vector<Point>& polygon,
                const Point& point);

        static bool isFootprintSafe(
                const std::vector<Point>& footprint,
                const PointCloud& points);

        static Point getTransformedPoint(
                const Pose2d& tf,
                const Point& pt);

        static std::vector<Point> getTransformedFootprint(
                const Pose2d& tf,
                const std::vector<Point>& footprint);

        static std::vector<Pose2d> getTrajectory(
                const Pose2d& vel,
                size_t num_of_poses,
                float future_time);

        static std::vector<float> multiplyMatrixToVector(
                const std::vector<float>& mat_a,
                const std::vector<float>& vec_b);

        static std::vector<float> multiplyMatrices(
                const std::vector<float>& mat_a,
                const std::vector<float>& mat_b,
                size_t N);

        static void transformPoint(
                const std::vector<float>& tf_mat,
                Point& pt);

        static std::vector<float> get2DTransformMat(
                float x,
                float y,
                float theta);

        static std::vector<float> getTransformMat(
                float x,
                float y,
                float z,
                float roll,
                float pitch,
                float yaw);

        static float getShortestAngle(
                float angle1,
                float angle2);

        static void findPerpendicularLineAt(
                float m,
                float c,
                const Point& p,
                float& perpendicular_m,
                float& perpendicular_c);

        static float distToLineSquared(
                float m,
                float c,
                const Point& p);

        static Point getProjectedPointOnLine(
                float m,
                float c,
                const Point& p);

        static Point getProjectedPointOnSegment(
                const Point& a,
                const Point& b,
                const Point& p,
                bool is_segment);

        static Point getProjectedPointOnMajorAxis(
                float m,
                float c,
                const Point& p);

        /*
         * find the distance of point p from a line formed by points a and b
         * source: https://stackoverflow.com/a/1501725/10460994
         */
        static float distToLineSquared(
                const Point& a,
                const Point& b,
                const Point& p,
                bool is_segment = false);

        static float distToLineSegmentSquared(
                const LineSegment& line_segment,
                const Point& p);

        static float fitLineRANSAC(
                const PointCloud& pts,
                unsigned start_index,
                unsigned end_index,
                float& m,
                float& c,
                float delta = 0.2f,
                size_t itr_limit = 10);

        static float fitLineRANSAC(
                const PointCloud& pts,
                float& m, float& c,
                float delta = 0.2f,
                size_t itr_limit = 10);

        static float fitLineSegmentRANSAC(
                const PointCloud& pts,
                unsigned start_index,
                unsigned end_index,
                LineSegment& line_segment,
                float delta = 0.2f,
                size_t itr_limit = 10);

        static float fitLineSegmentRANSAC(
                const PointCloud& pts,
                LineSegment& line_segment,
                float delta = 0.2f,
                size_t itr_limit = 10);

        static std::vector<LineSegment> fitLineSegmentsRANSAC(
                const PointCloud& pts,
                float score_threshold = 0.9f,
                float delta = 0.2f,
                size_t itr_limit = 10);

        static float fitLineRegression(
                const PointCloud& pts,
                unsigned start_index,
                unsigned end_index,
                LineSegment& line_segment);

        static float fitLineRegression(
                const PointCloud& pts,
                LineSegment& line_segment);

        static std::vector<LineSegment> piecewiseRegression(
                const PointCloud& pts,
                float error_threshold = 0.1f);

        static std::vector<LineSegment> piecewiseRegressionSplit(
                const PointCloud& pts,
                float error_threshold = 0.1f);

        static void mergeCloseLines(
                std::vector<LineSegment>& line_segments,
                float distance_threshold = 0.2,
                float angle_threshold = 0.2);

        static void mergeCloseLinesBF(
                std::vector<LineSegment>& line_segments,
                float distance_threshold = 0.2,
                float angle_threshold = 0.2);

        static std::vector<LineSegment> fitLineSegments(
                const std::vector<Point> &pts,
                float regression_error_threshold = 0.1f,
                float distance_threshold = 0.2,
                float angle_threshold = 0.2);

        static float clip(
                float value,
                float max_limit,
                float min_limit);

        static float signedClip(
                float value,
                float max_limit,
                float min_limit);

        static float clipAngle(
                float raw_angle);

        static Pose2d applyVelLimits(
                const Pose2d& vel,
                const Pose2d& max_vel,
                const Pose2d& min_vel);

        static Pose2d applyAccLimits(
                const Pose2d& cmd_vel,
                const Pose2d& curr_vel,
                const Pose2d& max_acc,
                float loop_rate);

        static float linearInterpolate(
                float src,
                float target,
                float t);

        static sensor_msgs::PointCloud convertToROSPC(
                const PointCloud& pc,
                const std::string& frame);

        static std::vector<Point> convertFromROSPC(
                const sensor_msgs::PointCloud& pc);

        static std::vector<Point> convertFromROSPC(
                const sensor_msgs::PointCloud2& pc);

        static PointCloud convertFromROSScan(
                const sensor_msgs::LaserScan& scan);

        static std::vector<float> getInverted2DTransformMat(
                const Pose2d& tf);

        static std::vector<float> getInverted2DTransformMat(
                const std::vector<float>& tf);

        static Pose2d getInverted2DTransformPose(
                const Pose2d& tf);

        static float getPerpendicularAngle(
                float angle);

        static float getReverseAngle(
                float angle);

        /*
         * Since the angles wrap around, the rule of thumb here is that the sector
         * considered is the one where you move counter-clockwise from min_angle and
         * all angles are always represented from -pi to pi
         */
        static bool isAngleWithinBounds(
                float angle,
                float max_angle,
                float min_angle);

        static std::vector<Point> generatePerpendicularPoints(
                const Pose2d& start,
                const Pose2d& end,
                const Point& pt,
                float max_perp_dist = 3.0f,
                float step_size = 0.1f);

        static nav_msgs::Path getPathMsgFromTrajectory(
                const std::vector<Pose2d>& trajectory,
                const std::string& frame);

        static visualization_msgs::Marker getPolygonAsMarker(
                const std::vector<Point>& polygon,
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.05f);

        static visualization_msgs::Marker getGeometricPathAsMarker(
                const std::vector<Pose2d>& geometric_path,
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.05f);

        static visualization_msgs::Marker getPointcloudAsMarker(
                const PointCloud& cloud,
                const std::string& frame,
                float diameter = 0.05f,
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f);

        static std::string getMatrixAsString(
                const std::vector<float>& mat,
                size_t N);
};

} // namespace geometry_common

#endif // GEOMETRY_COMMON_UTILS_H

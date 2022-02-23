#include <geometry_common/utils.h>
#include <math.h>
#include <cassert>
#include <list>
#include <deque>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace geometry_common
{

float Utils::roundFloat(
        float value,
        unsigned decimal_places)
{
    unsigned multiplier = pow(10, decimal_places);
    return ((float)((int)round(value * multiplier))) / multiplier;
}

Point Utils::getMeanPoint(
        const PointCloud& points,
        unsigned start_index,
        unsigned end_index)
{
    Point sum_pt(0.0f, 0.0f, 0.0f);
    for ( size_t i = start_index; i <= end_index; i++ )
    {
        sum_pt = sum_pt + points[i];
    }
    return sum_pt * (1.0f/(end_index-start_index+1));
}

Point Utils::getMeanPoint(
        const PointCloud& points)
{
    return Utils::getMeanPoint(points, 0, points.size()-1);
}

template <typename T>
Pose2d Utils::getMeanPose(
        const T& poses)
{
    Pose2d mean_cart_pose;
    if ( poses.size() == 0 )
    {
        return mean_cart_pose;
    }
    float cos_theta_sum = 0.0f;
    float sin_theta_sum = 0.0f;
    for ( const Pose2d& p : poses )
    {
        mean_cart_pose.x += p.x;
        mean_cart_pose.y += p.y;
        cos_theta_sum += cos(p.theta);
        sin_theta_sum += sin(p.theta);
    }
    mean_cart_pose.x /= poses.size();
    mean_cart_pose.y /= poses.size();
    mean_cart_pose.theta = atan2(sin_theta_sum/poses.size(),
                                 cos_theta_sum/poses.size());
    return mean_cart_pose;
}
template Pose2d Utils::getMeanPose(const std::vector<Pose2d>& poses);
template Pose2d Utils::getMeanPose(const std::deque<Pose2d>& poses);
template Pose2d Utils::getMeanPose(const std::list<Pose2d>& poses);

Point Utils::getClosestPoint(
        const PointCloud& points,
        float x,
        float y,
        float z)
{
    Point min_pt(1e6f, 1e6f, 1e6f);
    float min_dist_sq = 1e8f;
    Point center(x, y, z);
    for ( const Point& p : points )
    {
        float dist_sq = p.getCartDistSquared(center);
        if ( dist_sq < min_dist_sq )
        {
            min_pt = p;
            min_dist_sq = dist_sq;
        }
    }
    return min_pt;
}

std::vector<std::vector<Point> > Utils::clusterPoints(
        const PointCloud& points,
        float cluster_distance_threshold,
        size_t min_cluster_size)
{
    std::vector<std::vector<Point> > clusters;

    /* populate remaining_points */
    std::list<Point> remaining_points;
    for ( const Point& p : points )
    {
        remaining_points.push_back(Point(p));
    }

    float threshold_dist_sq = pow(cluster_distance_threshold, 2);

    /* cluster remaining_points iteratively */
    while ( !remaining_points.empty() )
    {
        std::vector<Point> cluster;
        std::list<Point> fringe;
        fringe.push_back(remaining_points.front());
        remaining_points.pop_front();

        while ( !fringe.empty() )
        {
            // get first point from fringe
            Point point = fringe.front();
            fringe.pop_front();
            cluster.push_back(point);

            // iterate over remaining points
            auto pt = remaining_points.begin();
            while ( pt != remaining_points.end() )
            {
                if ( point.getCartDistSquared(*pt) < threshold_dist_sq )
                {
                    fringe.push_back(*pt);
                    pt = remaining_points.erase(pt);
                    continue;
                }
                ++pt;
            }
        }
        if ( cluster.size() > min_cluster_size )
        {
            clusters.push_back(cluster);
        }
    }
    return clusters;
}

std::vector<std::vector<Point> > Utils::clusterOrderedPoints(
        const PointCloud& points,
        float cluster_distance_threshold,
        size_t min_cluster_size)
{
    std::vector<std::vector<Point> > clusters;

    /* populate remaining_points */
    std::list<Point> remaining_points;
    for ( Point p : points )
    {
        remaining_points.push_back(Point(p));
    }

    float threshold_dist_sq = pow(cluster_distance_threshold, 2);

    while ( !remaining_points.empty() )
    {
        std::vector<Point> cluster;
        cluster.push_back(remaining_points.front());
        remaining_points.pop_front();

        // iterate over remaining points
        auto pt = remaining_points.begin();
        while ( pt != remaining_points.end() )
        {
            if ( cluster.back().getCartDistSquared(*pt) < threshold_dist_sq )
            {
                cluster.push_back(*pt);
                pt = remaining_points.erase(pt);
                continue;
            }
            ++pt;
        }
        if ( cluster.size() > min_cluster_size )
        {
            clusters.push_back(cluster);
        }
    }

    /* for 360 degree laser points */
    if ( clusters.size() > 1 &&
         clusters.front().front().getCartDistSquared(clusters.back().back()) < threshold_dist_sq )
    {
        clusters.front().reserve(clusters.front().size() + clusters.back().size());
        clusters.front().insert(clusters.front().begin(), clusters.back().begin(),
                                clusters.back().end());
        clusters.pop_back();
    }
    return clusters;
}

bool Utils::isPointInPolygon(
        const std::vector<Point>& polygon,
        const Point& point)
{
    size_t i, j, counter = 0;
    for (i = 0, j = polygon.size()-1; i < polygon.size(); j = i++)
    {
        if ( ((polygon[i].y > point.y) != (polygon[j].y > point.y)) &&
             (point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x) )
        {
            counter++;
        }
    }
    return (counter % 2 == 1);
}

bool Utils::isFootprintSafe(
        const std::vector<Point>& footprint,
        const PointCloud& points)
{
    for ( const Point& p : points )
    {
        if ( Utils::isPointInPolygon(footprint, p) )
        {
            return false;
        }
    }
    return true;
}

Point Utils::getTransformedPoint(
        const Pose2d& tf,
        const Point& pt)
{
    Point transformed_pt = pt;
    transformed_pt.x = (cos(tf.theta) * pt.x) + (-sin(tf.theta) * pt.y) + tf.x;
    transformed_pt.y = (sin(tf.theta) * pt.x) + (cos(tf.theta) * pt.y) + tf.y;
    transformed_pt.z = 0.0f;
    return transformed_pt;
}

Point Utils::getTransformedPoint(
        const std::vector<float>& tf_mat,
        const Point& pt)
{
    Point transformed_pt(pt);
    assert(tf_mat.size() == 16 || tf_mat.size() == 9);
    if ( tf_mat.size() == 16 )
    {
        Utils::transformPoint(tf_mat, transformed_pt);
    }
    else
    {
        Utils::transformPoint2D(tf_mat, transformed_pt);
    }
    return transformed_pt;
}

void Utils::transformPoint2D(
        const std::vector<float>& tf_mat,
        Point& pt)
{
    assert(tf_mat.size() == 9);
    std::vector<float> p_vec{pt.x, pt.y, 1.0f};
    std::vector<float> transformed_p_vec = Utils::multiplyMatrixToVector(tf_mat, p_vec);
    pt.x = transformed_p_vec[0];
    pt.y = transformed_p_vec[1];
}

void Utils::transformPoint(
        const std::vector<float>& tf_mat,
        Point& pt)
{
    assert(tf_mat.size() == 16);
    std::vector<float> p_vec{pt.x, pt.y, pt.z, 1.0f};
    std::vector<float> transformed_p_vec = Utils::multiplyMatrixToVector(tf_mat, p_vec);
    pt.x = transformed_p_vec[0];
    pt.y = transformed_p_vec[1];
    pt.z = transformed_p_vec[2];
}

Pose2d Utils::getTransformedPose(
        const std::vector<float> tf_mat,
        const Pose2d& pose)
{
    assert(tf_mat.size() == 9);
    return Pose2d(Utils::multiplyMatrices(tf_mat, pose.getMat(), 3));
}

Pose2d Utils::getTransformedPose(
        const Pose2d& tf,
        const Pose2d& pose)
{
    return Utils::getTransformedPose(tf.getMat(), pose);
}

std::vector<Point> Utils::getTransformedFootprint(
        const Pose2d& tf,
        const std::vector<Point>& footprint)
{ 
    std::vector<Point> fp;
    fp.reserve(footprint.size());
    for ( const Point& pt : footprint )
    {
        fp.push_back(Utils::getTransformedPoint(tf, pt));
    }
    return footprint;
}

std::vector<Pose2d> Utils::getTrajectory(
        const Pose2d& vel,
        size_t num_of_poses,
        float future_time)
{
    std::vector<Pose2d> traj;
    traj.reserve(num_of_poses+1);

    float delta_t = future_time/num_of_poses;

    Pose2d tf = vel * delta_t;
    std::vector<float> tf_mat = tf.getMat();
    std::vector<float> pos_mat = Pose2d().getMat(); // identity mat

    /* add current pose (for extra safety) */
    traj.push_back(Pose2d());

    for ( size_t i = 0; i < num_of_poses; i++ )
    {
        pos_mat = Utils::multiplyMatrices(pos_mat, tf_mat, 3);
        traj.push_back(Pose2d(pos_mat));
    }
    return traj;
}

std::vector<float> Utils::multiplyMatrixToVector(
        const std::vector<float>& mat_a,
        const std::vector<float>& vec_b)
{
    size_t N = vec_b.size();
    assert(N * N == mat_a.size());
    std::vector<float> vec_c(N, 0.0f);
    for ( size_t i = 0; i < N; i++ )
    {
        for ( size_t j = 0; j < N; j++ )
        {
            vec_c[i] += mat_a[i*N + j] * vec_b[j];
        }
    }
    return vec_c;
}

std::vector<float> Utils::multiplyMatrices(
        const std::vector<float>& mat_a,
        const std::vector<float>& mat_b,
        size_t N)
{
    assert(N * N == mat_a.size() && N * N == mat_b.size());
    std::vector<float> mat_c(N*N, 0.0f);
    for ( size_t i = 0; i < N; i++ )
    {
        for ( size_t j = 0; j < N; j++ )
        {
            for ( size_t k = 0; k < N; k++ )
            {
                mat_c[i*N + j] += mat_a[i*N + k] * mat_b[k*N + j];
            }
        }
    }
    return mat_c;
}

std::vector<float> Utils::get2DTransformMat(
        float x,
        float y,
        float theta)
{
    return std::vector<float>
           {cos(theta), -sin(theta), x,
            sin(theta),  cos(theta), y,
            0.0f,        0.0f,       1.0f};
}

std::vector<float> Utils::getTransformMat(
        float x,
        float y,
        float z,
        float roll,
        float pitch,
        float yaw)
{
    std::vector<float> tf_mat(16, 0.0f);
    /* rotation */
    tf_mat[0] = cos(yaw) * cos(pitch);
    tf_mat[1] = (cos(yaw) * sin(pitch) * sin(roll)) - (sin(yaw) * cos(roll));
    tf_mat[2] = (cos(yaw) * sin(pitch) * cos(roll)) + (sin(yaw) * sin(roll));
    tf_mat[4] = sin(yaw)* cos(pitch);
    tf_mat[5] = (sin(yaw) * sin(pitch) * sin(roll)) + (cos(yaw) * cos(roll));
    tf_mat[6] = (sin(yaw) * sin(pitch) * cos(roll)) - (cos(yaw) * sin(roll));
    tf_mat[8] = -sin(pitch);
    tf_mat[9] = cos(pitch) * sin(roll);
    tf_mat[10] = cos(pitch) * cos(roll);

    /* translation */
    tf_mat[3] = x;
    tf_mat[7] = y;
    tf_mat[11] = z;

    tf_mat[15] = 1.0f;
    return tf_mat;
}

float Utils::getShortestAngle(
        float angle1,
        float angle2)
{
    return atan2(sin(angle1 - angle2), cos(angle1 - angle2));
}

void Utils::findPerpendicularLineAt(
        float m,
        float c,
        const Point& p,
        float& perpendicular_m,
        float& perpendicular_c)
{
    perpendicular_m = ( fabs(m) < 1e-8f ) ? 1e8f : -1/m;
    perpendicular_c = p.y - (perpendicular_m * p.x);
}

float Utils::distToLineSquared(
        float m,
        float c,
        const Point& p)
{
    Point proj_pt = Utils::getProjectedPointOnLine(m, c, p);
    return p.getCartDistSquared(proj_pt);
}

Point Utils::getProjectedPointOnLine(
        float m,
        float c,
        const Point& p)
{
    float perpendicular_m, perpendicular_c;
    Utils::findPerpendicularLineAt(m, c, p, perpendicular_m, perpendicular_c);
    Point proj_pt;
    proj_pt.x = (perpendicular_c - c) / (m - perpendicular_m);
    proj_pt.y = (m * proj_pt.x) + c;
    return proj_pt;
}

Point Utils::getProjectedPointOnSegment(
        const Point& a,
        const Point& b,
        const Point& p,
        bool is_segment)
{
    Point proj_pt;
    float length_sq = a.getCartDistSquared(b);
    if ( length_sq == 0.0f )
    {
        proj_pt = Point(a);
        return proj_pt;
    }
    float t = ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / length_sq;
    if ( is_segment )
    {
        t = Utils::clip(t, 1.0f, 0.0f);
    }
    proj_pt.x = a.x + t * (b.x - a.x);
    proj_pt.y = a.y + t * (b.y - a.y);
    return proj_pt;
}

Point Utils::getProjectedPointOnMajorAxis(
        float m,
        float c,
        const Point& p)
{
    Point proj_pt;
    bool major_axis_x = ( fabs(m) < 1.0f );
    if ( major_axis_x )
    {
        proj_pt.x = p.x;
        proj_pt.y = (m * p.x) + c;
    }
    else
    {
        proj_pt.y = p.y;
        proj_pt.x = (p.y - c) / m;
    }
    return proj_pt;
}

float Utils::distToLineSquared(
        const Point& a,
        const Point& b,
        const Point& p,
        bool is_segment)
{
    return p.getCartDistSquared(Utils::getProjectedPointOnSegment(a, b, p, is_segment));
}

float Utils::distToLineSegmentSquared(
        const LineSegment& line_segment,
        const Point& p)
{
    return Utils::distToLineSquared(line_segment.start, line_segment.end, p, true);
}

float Utils::fitLineRANSAC(
        const PointCloud& pts,
        unsigned start_index,
        unsigned end_index,
        float& m,
        float& c,
        float delta,
        size_t itr_limit)
{
    if ( end_index <= start_index )
    {
        m = 0;
        c = 0;
        return 0.0f;
    }

    unsigned max_score = 0;
    size_t index_1 = start_index;
    size_t index_2 = end_index;
    size_t num_of_points = end_index-start_index+1;
    float delta_sq = delta * delta;
    for ( size_t itr_num = 0; itr_num < itr_limit; ++itr_num )
    {
        size_t ind_1 = (rand() % num_of_points) + start_index;
        size_t ind_2 = (rand() % num_of_points) + start_index;
        unsigned score = 0;
        for (Point p : pts)
        {
            if ( Utils::distToLineSquared(pts[ind_1], pts[ind_2], p) < delta_sq )
            {
                score ++;
            }
        }
        if ( score > max_score )
        {
            max_score = score;
            index_1 = ind_1;
            index_2 = ind_2;
        }
    }

    float dx = pts[index_1].x - pts[index_2].x;
    if ( fabs(dx) < 1e-8 )
    {
        dx = 1e-8;
    }
    m = (pts[index_1].y - pts[index_2].y) / dx;
    c = pts[index_1].y - m*pts[index_1].x;
    return (float)max_score/num_of_points;
}

float Utils::fitLineRANSAC(
        const PointCloud& pts,
        float &m,
        float &c,
        float delta,
        size_t itr_limit)
{
    return Utils::fitLineRANSAC(pts, 0, pts.size()-1, m, c, delta, itr_limit);
}

float Utils::fitLineSegmentRANSAC(
        const PointCloud& pts,
        unsigned start_index,
        unsigned end_index,
        LineSegment& line_segment,
        float delta,
        size_t itr_limit)
{
    float m, c, score;
    float delta_sq = delta * delta;
    score = Utils::fitLineRANSAC(pts, start_index, end_index, m, c, delta, itr_limit);
    line_segment.start.x = 1e6f;
    line_segment.start.y = 1e6f;
    line_segment.end.x = -1e6f;
    line_segment.end.y = -1e6f;
    bool major_axis_x = ( fabs(m) < 1.0f );
    for ( size_t i = start_index; i <= end_index; i++ )
    {
        if ( Utils::distToLineSquared(m, c, pts[i]) < delta_sq )
        {
            if ( major_axis_x )
            {
                if ( pts[i].x < line_segment.start.x )
                {
                    line_segment.start = Utils::getProjectedPointOnMajorAxis(m, c, pts[i]);
                }
                if ( pts[i].x > line_segment.end.x )
                {
                    line_segment.end = Utils::getProjectedPointOnMajorAxis(m, c, pts[i]);
                }
            }
            else
            {
                if ( pts[i].y < line_segment.start.y )
                {
                    line_segment.start = Utils::getProjectedPointOnMajorAxis(m, c, pts[i]);
                }
                if ( pts[i].y > line_segment.end.y )
                {
                    line_segment.end = Utils::getProjectedPointOnMajorAxis(m, c, pts[i]);
                }
            }
        }
    }
    return score;
}

float Utils::fitLineSegmentRANSAC(
        const PointCloud& pts,
        LineSegment& line_segment,
        float delta,
        size_t itr_limit)
{
    return Utils::fitLineSegmentRANSAC(pts, 0, pts.size()-1, line_segment, delta, itr_limit);
}

std::vector<LineSegment> Utils::fitLineSegmentsRANSAC(
        const PointCloud& pts,
        float score_threshold,
        float delta,
        size_t itr_limit)
{
    struct RegressionLineSegment
    {
        unsigned start_index, end_index;
        LineSegment line_segment;
    };

    std::vector<LineSegment> line_segments;
    if (pts.size() < 2)
    {
        return line_segments;
    }

    /* fill in the initial segments */
    std::vector<RegressionLineSegment> segments(1);
    segments[0].start_index = 0;
    segments[0].end_index = pts.size()-1;

    float score = Utils::fitLineSegmentRANSAC(pts, segments[0].start_index,
                                              segments[0].end_index, 
                                              segments[0].line_segment,
                                              delta, itr_limit);
    if ( score > score_threshold )
    {
        line_segments.push_back(segments[0].line_segment);
        return line_segments;
    }

    std::vector<float> scores;
    scores.push_back(score);

    while ( true )
    {
        /* find segment with minimum error */
        size_t i = 0;
        float lowest_score = 0.0f;
        for ( size_t j = 0; j < scores.size(); j++ )
        {
            float score = scores[j];
            if ( score < lowest_score )
            {
                lowest_score = score;
                i = j;
            }
        }

        /* if segment with lowest score is within score_threshold, stop splitting */
        if ( lowest_score > score_threshold )
        {
            break;
        }

        /* if segment with lowest_score is only 2 points, stop splitting */
        if ( segments[i].end_index - segments[i].start_index < 3 )
        {
            break;
        }

        /* split the worst segments and update scores */
        size_t split_pt_index = segments[i].start_index;
        float max_dist = 0.0f;
        for ( size_t j = segments[i].start_index+1; j+1 <= segments[i].end_index; j++ )
        {
            // float dist = Utils::distToLineSegmentSquared(segments[i].line_segment, pts[j]);
            float dist = Utils::distToLineSquared(pts[segments[i].start_index],
                                                  pts[segments[i].end_index],
                                                  pts[j], true);
            // std::cout << j << " " << dist << std::endl;
            if ( dist > max_dist )
            {
                max_dist = dist;
                split_pt_index = j;
            }
        }
        RegressionLineSegment new_segment;
        new_segment.start_index = segments[i].start_index;
        new_segment.end_index = split_pt_index;
        segments[i].start_index = split_pt_index+1;
        segments.insert(segments.begin() + i, new_segment);
        scores.insert(scores.begin() + i, 0.0f);

        scores[i] = Utils::fitLineSegmentRANSAC(pts, segments[i].start_index,
                                                segments[i].end_index,
                                                segments[i].line_segment,
                                                delta, itr_limit);
        scores[i+1] = Utils::fitLineSegmentRANSAC(pts, segments[i+1].start_index,
                                                  segments[i+1].end_index,
                                                  segments[i+1].line_segment,
                                                  delta, itr_limit);
    }

    line_segments.clear();
    /* create line_segments from regression line segments */
    for ( RegressionLineSegment rls : segments )
    {
        line_segments.push_back(rls.line_segment);
    }
    return line_segments;
}

float Utils::fitLineRegression(
        const PointCloud& pts,
        unsigned start_index,
        unsigned end_index,
        LineSegment& line_segment)
{
    if ( pts.size() < 2 ||
         start_index >= pts.size() || end_index >= pts.size() ||
         end_index <= start_index || end_index - start_index < 2 )
    {
        line_segment = LineSegment();
        return 0.0f;
    }
    Point mean_pt = Utils::getMeanPoint(pts, start_index, end_index);

    float dx = pts[end_index].x - pts[start_index].x;
    float dy = pts[end_index].y - pts[start_index].y;
    bool swap_axis = ( fabs(dx) < fabs(dy) );
    float numerator = 0.0f;
    float denominator = 0.0f;
    for ( size_t i = start_index; i <= end_index; i++ )
    {
        numerator += (pts[i].x - mean_pt.x) * (pts[i].y - mean_pt.y);
        if ( !swap_axis )
        {
            denominator += pow(pts[i].x - mean_pt.x, 2);
        }
        else
        {
            denominator += pow(pts[i].y - mean_pt.y, 2);
        }
    }
    if ( denominator < 1e-8 )
    {
        denominator = 1e-8;
    }

    if ( !swap_axis )
    {
        float m = numerator / denominator;
        float c = mean_pt.y - (m * mean_pt.x);
        line_segment.start.x = pts[start_index].x;
        line_segment.start.y = (m * pts[start_index].x) + c;
        line_segment.end.x = pts[end_index].x;
        line_segment.end.y = (m * pts[end_index].x) + c;
    }
    else
    {
        float n = numerator / denominator;
        float d = mean_pt.x - (n * mean_pt.y);
        line_segment.start.y = pts[start_index].y;
        line_segment.start.x = (n * pts[start_index].y) + d;
        line_segment.end.y = pts[end_index].y;
        line_segment.end.x = (n * pts[end_index].y) + d;
    }

    float error = 0.0f;
    for ( size_t i = start_index; i <= end_index; i++ )
    {
        error += Utils::distToLineSegmentSquared(line_segment, pts[i]);
    }
    return error;
}

float Utils::fitLineRegression(
        const PointCloud& pts,
        LineSegment& line_segment)
{
    return Utils::fitLineRegression(pts, 0, pts.size()-1, line_segment);
}

std::vector<LineSegment> Utils::piecewiseRegression(
        const PointCloud& pts,
        float error_threshold)
{
    struct RegressionLineSegment
    {
        unsigned start_index, end_index;
    };

    std::vector<LineSegment> line_segments;
    if (pts.size() < 2)
    {
        return line_segments;
    }

    /* fill in the initial segments */
    std::vector<RegressionLineSegment> segments(pts.size() / 2);
    for ( size_t i = 0; i < segments.size(); i++ )
    {
        segments[i].start_index = 2*i;
        segments[i].end_index = 2*i + 1;
        /* if pts has odd num of points, last segment should have 3 points */
        if ( (2*i)+3 >= pts.size() )
        {
            segments[i].end_index = pts.size() - 1;
        }
    }

    /* errors when 2 consecutive segments are merged */
    std::vector<float> errors(segments.size()-1);
    LineSegment line_segment; // not used;
    for ( size_t i = 0; i < errors.size(); i++ )
    {
        errors[i] = Utils::fitLineRegression(pts, segments[i].start_index,
                                             segments[i+1].end_index, line_segment);
    }

    while ( segments.size() > 1 )
    {
        /* find consecutive segments with minimum error if merged */
        size_t i = 0;
        float lowest_error = 1e6f; // just a large number
        for ( size_t j = 0; j < errors.size(); j++ )
        {
            float error = errors[j];
            if ( error < lowest_error )
            {
                lowest_error = error;
                i = j;
            }
        }

        /* if merged_segment with least error violates error_threshold, stop merging */
        if ( lowest_error > error_threshold )
        {
            break;
        }

        /* merge the best segments and update errors */
        segments[i].end_index = segments[i+1].end_index;
        segments.erase(segments.begin() + i + 1);

        LineSegment line_segment; // not used
        if (i > 0)
        {
            errors[i-1] = Utils::fitLineRegression(pts, segments[i-1].start_index,
                                                   segments[i].end_index, line_segment);
        }
        if (i < segments.size() - 1)
        {
            errors[i+1] = Utils::fitLineRegression(pts, segments[i].start_index,
                                                   segments[i+1].end_index, line_segment);
        }
        errors.erase(errors.begin() + i);	
    }

    /* create line_segments from regression line segments */
    for ( RegressionLineSegment rls : segments )
    {
        LineSegment l;
        Utils::fitLineRegression(pts, rls.start_index, rls.end_index, l);
        line_segments.push_back(l);
    }
    return line_segments;
}

std::vector<LineSegment> Utils::piecewiseRegressionSplit(
        const PointCloud& pts,
        float error_threshold)
{
    struct RegressionLineSegment
    {
        unsigned start_index, end_index;
        LineSegment line_segment;
    };

    std::vector<LineSegment> line_segments;
    if (pts.size() < 2)
    {
        return line_segments;
    }

    /* fill in the initial segments */
    std::vector<RegressionLineSegment> segments(1);
    segments[0].start_index = 0;
    segments[0].end_index = pts.size()-1;

    float error = Utils::fitLineRegression(pts, segments[0].start_index,
                                           segments[0].end_index,
                                           segments[0].line_segment);
    if ( error < error_threshold )
    {
        line_segments.push_back(segments[0].line_segment);
        return line_segments;
    }

    std::vector<float> errors;
    errors.push_back(error);

    while ( true )
    {
        /* find segment with maximum error */
        size_t i = 0;
        float highest_error = 0.0f;
        for ( size_t j = 0; j < errors.size(); j++ )
        {
            float error = errors[j];
            if ( error > highest_error )
            {
                highest_error = error;
                i = j;
            }
        }

        /* if segment with highest error is within error_threshold, stop splitting */
        if ( highest_error < error_threshold )
        {
            break;
        }

        /* if segment with highest error is only 2 points, stop splitting */
        if ( segments[i].end_index - segments[i].start_index < 3 )
        {
            break;
        }

        /* split the worst segments and update errors */
        size_t split_pt_index = segments[i].start_index;
        float max_dist = 0.0f;
        for ( size_t j = segments[i].start_index+1; j+1 <= segments[i].end_index; j++ )
        {
            // float dist = Utils::distToLineSegmentSquared(segments[i].line_segment,
            //                                              pts[j]);
            float dist = Utils::distToLineSquared(pts[segments[i].start_index],
                                                  pts[segments[i].end_index],
                                                  pts[j], true);
            if ( dist > max_dist )
            {
                max_dist = dist;
                split_pt_index = j;
            }
        }
        RegressionLineSegment new_segment;
        new_segment.start_index = segments[i].start_index;
        new_segment.end_index = split_pt_index;
        segments[i].start_index = split_pt_index+1;
        segments.insert(segments.begin() + i, new_segment);
        errors.insert(errors.begin() + i, 0.0f);

        errors[i] = Utils::fitLineRegression(pts, segments[i].start_index,
                                               segments[i].end_index, segments[i].line_segment);
        errors[i+1] = Utils::fitLineRegression(pts, segments[i+1].start_index,
                                               segments[i+1].end_index, segments[i+1].line_segment);
    }

    /* create line_segments from regression line segments */
    for ( RegressionLineSegment rls : segments )
    {
        line_segments.push_back(rls.line_segment);
    }
    return line_segments;
}

void Utils::mergeCloseLines(
        std::vector<LineSegment>& line_segments,
        float distance_threshold,
        float angle_threshold)
{
	if (line_segments.size() < 2)
    {
		return;
    }

    size_t i = 0;

    while ( i < line_segments.size()-1 )
    {
        float linear_dist = line_segments[i].end.getCartDist(line_segments[i+1].start);
        float angular_dist = Utils::getShortestAngle(line_segments[i].getAngle(),
                                                     line_segments[i+1].getAngle());
        if ( linear_dist < distance_threshold && fabs(angular_dist) < angle_threshold )
        {
            line_segments[i].end = line_segments[i+1].end;
            line_segments.erase(line_segments.begin() + i + 1);
            continue;
        }
        i++;
    }
}

void Utils::mergeCloseLinesBF(
        std::vector<LineSegment>& line_segments,
        float distance_threshold,
        float angle_threshold)
{
	if (line_segments.size() < 2)
    {
		return;
    }

    size_t skip_index = 1;

    while ( skip_index < line_segments.size() )
    {
        size_t i = 0;
        while ( i+skip_index < line_segments.size() )
        {
            float linear_dist = line_segments[i].end.getCartDist(line_segments[i+skip_index].start);
            float angular_dist = Utils::getShortestAngle(line_segments[i].getAngle(),
                                                         line_segments[i+skip_index].getAngle());
            if ( linear_dist < distance_threshold && fabs(angular_dist) < angle_threshold )
            {
                line_segments[i].end = line_segments[i+skip_index].end;
                line_segments.erase(line_segments.begin() + i + skip_index);
                continue;
            }
            i++;
        }
        skip_index ++;
    }
}

std::vector<LineSegment> Utils::fitLineSegments(
        const PointCloud& pts,
        float regression_error_threshold,
        float distance_threshold,
        float angle_threshold)
{
    std::vector<LineSegment> lines = Utils::piecewiseRegression(pts, regression_error_threshold);
    Utils::mergeCloseLines(lines, distance_threshold, angle_threshold);
    return lines;
}

float Utils::clip(
        float value,
        float max_limit,
        float min_limit)
{
    return std::max(std::min(value, max_limit), min_limit);
}

float Utils::signedClip(
        float value,
        float max_limit,
        float min_limit)
{
    int sign = ( value < 0.0f ) ? -1 : 1;
    return sign * Utils::clip(fabsf(value), max_limit, min_limit);
}

float Utils::clipAngle(
        float raw_angle)
{
    float angle = raw_angle;
    if ( angle > M_PI )
    {
        angle -= 2*M_PI;
    }
    else if ( angle < -M_PI )
    {
        angle += 2*M_PI;
    }
    return angle;
}

Pose2d Utils::applyVelLimits(
        const Pose2d& vel,
        const Pose2d& max_vel,
        const Pose2d& min_vel)
{
    Pose2d clipped_vel;
    clipped_vel.x = Utils::clip(vel.x, max_vel.x, min_vel.x);
    clipped_vel.y = Utils::clip(vel.y, max_vel.y, min_vel.y);
    clipped_vel.theta = Utils::clip(vel.theta, max_vel.theta, min_vel.theta);
    return clipped_vel;
}

Pose2d Utils::applyAccLimits(
        const Pose2d& cmd_vel,
        const Pose2d& curr_vel,
        const Pose2d& max_acc,
        float loop_rate)
{
    Pose2d vel;
    Pose2d max_acc_per_loop = max_acc * (1.0f/loop_rate);
    vel.x = Utils::clip(cmd_vel.x,
                        curr_vel.x + max_acc_per_loop.x,
                        curr_vel.x - max_acc_per_loop.x);
    vel.y = Utils::clip(cmd_vel.y,
                        curr_vel.y + max_acc_per_loop.y,
                        curr_vel.y - max_acc_per_loop.y);
    vel.theta = Utils::clip(cmd_vel.theta,
                            curr_vel.theta + max_acc_per_loop.theta,
                            curr_vel.theta - max_acc_per_loop.theta);
    return vel;
}

float Utils::linearInterpolate(
        float src,
        float target,
        float t)
{
    return ( t >= 1.0f ) ? target :
           ( t <= 0.0f ) ? src    : (src * (1.0f - t)) + (target * t);
}

sensor_msgs::PointCloud Utils::convertToROSPC(
        const PointCloud& pc,
        const std::string& frame)
{
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = frame;
    cloud.points.reserve(pc.size());
    for ( const Point& pt : pc )
    {
        cloud.points.push_back(pt.getPoint32());
    }
    return cloud;
}

PointCloud Utils::convertFromROSPC(
        const sensor_msgs::PointCloud& pc)
{
    std::vector<Point> points;
    points.reserve(pc.points.size());
    for ( const geometry_msgs::Point32& p : pc.points )
    {
        points.push_back(Point(p));
    }
    return points;
}

PointCloud Utils::convertFromROSPC(
        const sensor_msgs::PointCloud2& cloud_msg,
        size_t row_sub_sample_factor,
        size_t col_sub_sample_factor)
{
    if ( cloud_msg.height == 0 || cloud_msg.width == 0 )
    {
        return std::vector<Point>();
    }
    PointCloud points;
    if ( cloud_msg.height == 1 ) // unorganised cloud
    {
        row_sub_sample_factor = 1;
    }
    points.reserve((cloud_msg.height / row_sub_sample_factor) *
                   (cloud_msg.width / col_sub_sample_factor));
    size_t col = 0, row = 0;
    size_t col_remainder = cloud_msg.width % col_sub_sample_factor;
    size_t row_skip_factor = (cloud_msg.width * (row_sub_sample_factor-1)) + col_remainder;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_msg, "z");
    while ( iter_x != iter_x.end() && row < cloud_msg.height )
    {
        if ( std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z) )
        {
            continue;
        }
        points.push_back(Point(*iter_x, *iter_y, *iter_z));

        col += col_sub_sample_factor;
        if ( col >= cloud_msg.width )
        {
            col = 0;
            row += row_sub_sample_factor;
            iter_x += row_skip_factor;
            iter_y += row_skip_factor;
            iter_z += row_skip_factor;
            continue;
        }
        iter_x += col_sub_sample_factor;
        iter_y += col_sub_sample_factor;
        iter_z += col_sub_sample_factor;
    }
    return points;
}

PointCloud Utils::convertFromROSScan(
        const sensor_msgs::LaserScan& scan)
{
    PointCloud laser_pts;
    for ( size_t i = 0; i < scan.ranges.size(); i++ )
    {
        if ( std::isnan(scan.ranges[i]) ||
             std::isinf(scan.ranges[i]) ||
             scan.ranges[i] >= scan.range_max ||
             scan.ranges[i] <= scan.range_min )
        {
            continue;
        }
        float angle = scan.angle_min + (i * scan.angle_increment);
        laser_pts.push_back(Point(scan.ranges[i] * cos(angle),
                                  scan.ranges[i] * sin(angle),
                                  0.0f));
    }
    return laser_pts;
}

std::vector<float> Utils::getInverted2DTransformMat(
        const Pose2d& tf)
{
    Pose2d inv_tf(tf);
    inv_tf.theta *= -1; // reverse angle
    std::vector<float> inv_mat = inv_tf.getMat();
    std::vector<float> p_vec{-tf.x, -tf.y, 0.0f};
    std::vector<float> transformed_vec = Utils::multiplyMatrixToVector(inv_mat, p_vec);
    inv_mat[2] = transformed_vec[0];
    inv_mat[5] = transformed_vec[1];
    return inv_mat;
}

std::vector<float> Utils::getInverted2DTransformMat(
        const std::vector<float>& tf)
{
    assert(tf.size() == 9);
    return Utils::getInverted2DTransformMat(Pose2d(tf));
}

Pose2d Utils::getInverted2DTransformPose(
        const Pose2d& tf)
{
    return Pose2d(Utils::getInverted2DTransformMat(tf));
}

float Utils::getPerpendicularAngle(
        float angle)
{
    float perpendicular_angle = angle + M_PI/2;
    if ( perpendicular_angle > M_PI )
    {
        perpendicular_angle -= 2*M_PI;
    }
    return perpendicular_angle;
}

float Utils::getReverseAngle(
        float angle)
{
    float reverse_angle = angle + M_PI;
    if ( reverse_angle > M_PI )
    {
        reverse_angle -= 2*M_PI;
    }
    return reverse_angle;
}

bool Utils::isAngleWithinBounds(
        float angle,
        float max_angle,
        float min_angle)
{
    return ( min_angle < max_angle )
           ? ( angle >= min_angle && angle <= max_angle )
           : ( angle <= min_angle && angle >= max_angle );
}

std::vector<Point> Utils::generatePerpendicularPoints(
        const Pose2d& start,
        const Pose2d& end,
        const Point& pt,
        float max_perp_dist,
        float step_size)
{
    float theta = atan2(end.y - start.y, end.x - start.x);
    float perpendicular_angle = Utils::getPerpendicularAngle(theta);

    Point unit_vec(cos(perpendicular_angle), sin(perpendicular_angle)); 
    std::vector<Point> pts;
    for ( float perp_dist = step_size; perp_dist < max_perp_dist; perp_dist += step_size )
    {
        Point offset = unit_vec * perp_dist;
        pts.push_back(pt + offset);
        pts.push_back(pt - offset);
    }
    return pts;
}

float Utils::getAngleBetweenPoints(
        const Point& a,
        const Point& b,
        const Point& c)
{
    Point vec_b_a = a - b;
    Point vec_b_c = c - b;
    return Utils::clipAngle(atan2(vec_b_c.y, vec_b_c.x) -
                            atan2(vec_b_a.y, vec_b_a.x));
}

bool Utils::isPolygonConvex(
        const std::vector<Point>& polygon)
{
    if ( polygon.size() <= 2 )
    {
        return true;
    }

    Point m = Utils::getMeanPoint(polygon);
    if ( !Utils::isPointInPolygon(polygon, m) )
    {
        return false;
    }

    for ( size_t i = 0; i < polygon.size(); i++ )
    {
        Point p1(polygon[i]);
        Point p2(polygon[(i+2)%polygon.size()]);
        m = (p1 + p2) * 0.5f;
        if ( !Utils::isPointInPolygon(polygon, m) )
        {
            return false;
        }
    }
    return true;
}

float Utils::calcPolygonArea(
        const std::vector<Point>& polygon)
{
    float area = 0.0f;
    size_t i, j;
    for ( i = 0; i < polygon.size(); i++ )
    {
        j = (i+1) % polygon.size();
        area += (polygon[i].x * polygon[j].y) - (polygon[i].y * polygon[j].x);
    }
    return area/2;
}

std::vector<Point> Utils::calcConvexHullOfPolygons(
        const std::vector<Point>& polygon_a,
        const std::vector<Point>& polygon_b)
{
    /* aggregate all points */
    std::vector<Point> pts;
    pts.reserve(pts.size() + polygon_a.size() + polygon_b.size());
    pts.insert(pts.end(), polygon_a.begin(), polygon_a.end());
    pts.insert(pts.end(), polygon_b.begin(), polygon_b.end());
    if ( pts.size() < 3 )
    {
        return pts;
    }

    /* find the lowest left most point */
    Point lower_left_pt(pts[0]);
    for ( size_t i = 0; i < pts.size(); i++ )
    {
        if ( pts[i].y < lower_left_pt.y )
        {
            lower_left_pt = pts[i];
        }
        else if ( pts[i].y == lower_left_pt.y && pts[i].x < lower_left_pt.x )
        {
            lower_left_pt = pts[i];
        }
    }

    /* sort points in increasing order of angle they and lower_left_pt makes with X axis */
    std::sort(pts.begin(), pts.end(),
              [&lower_left_pt](const Point& a, const Point& b)
              {
                  return atan2(a.y - lower_left_pt.y, a.x - lower_left_pt.x)
                       < atan2(b.y - lower_left_pt.y, b.x - lower_left_pt.x);
              });

    /* walk along pts and remove points that form non counter clockwise turn */
    std::vector<Point> convex_hull;
    for ( Point& p : pts )
    {
        while ( convex_hull.size() > 1 )
        {
            std::vector<Point>::const_iterator it = convex_hull.end();
            float angle = Utils::getAngleBetweenPoints(p, *(it-1), *(it-2));
            if ( angle > 0 ) // counter clockwise turn is allowed
            {
                break;
            }
            convex_hull.pop_back();
        }
        convex_hull.push_back(p);
    }
    return convex_hull;
}

nav_msgs::Path Utils::getPathMsgFromTrajectory(const std::vector<Pose2d>& trajectory,
                                               const std::string& frame)
{
    nav_msgs::Path path_msg;
    // path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = frame;
    path_msg.poses.clear();

    for ( Pose2d pose : trajectory )
    {
        path_msg.poses.push_back(pose.getPoseStamped(frame));
    }
    return path_msg;
}

visualization_msgs::Marker Utils::getGeometricPathAsMarker(
        const std::vector<Pose2d>& geometric_path,
        const std::string& frame,
        float red,
        float green,
        float blue,
        float alpha,
        float line_width)
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    // marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = line_width;
    marker.pose.orientation.w = 1.0f;
    marker.points.reserve(geometric_path.size());
    for ( size_t i = 0; i < geometric_path.size(); i++ )
    {
        geometry_msgs::Point pt;
        pt.x = geometric_path[i].x;
        pt.y = geometric_path[i].y;
        marker.points.push_back(pt);
    }
    return marker;
}

visualization_msgs::Marker Utils::getPointcloudAsMarker(
        const PointCloud& cloud,
        const std::string& frame,
        float diameter,
        float red,
        float green,
        float blue,
        float alpha)
{
    visualization_msgs::Marker cloud_marker;
    cloud_marker.type = visualization_msgs::Marker::POINTS;
    cloud_marker.pose.orientation.w = 1.0f;
    cloud_marker.scale.x = diameter;
    cloud_marker.scale.y = diameter;
    cloud_marker.color.r = red;
    cloud_marker.color.g = green;
    cloud_marker.color.b = blue;
    cloud_marker.color.a = alpha;
    cloud_marker.header.frame_id = frame;
    // cloud_marker.header.stamp = ros::Time::now();
    cloud_marker.points.reserve(cloud.size());
    for ( size_t i = 0; i < cloud.size(); i++ )
    {
        cloud_marker.points.push_back(cloud[i].getPoint());
    }
    return cloud_marker;
}

visualization_msgs::Marker Utils::getPolygonAsMarker(
        const std::vector<Point>& polygon,
        const std::string& frame,
        float red,
        float green,
        float blue,
        float alpha,
        float line_width)
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    // marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = line_width;
    marker.pose.orientation.w = 1.0f;
    size_t N = polygon.size();
    marker.points.reserve(N+1);
    for ( size_t i = 0; i < N+1; i++ )
    {
        marker.points.push_back(polygon[i%N].getPoint());
    }
    return marker;
}

visualization_msgs::Marker Utils::getStringAsMarker(
        const std::string& string_label,
        const std::string& frame,
        float red,
        float green,
        float blue,
        float alpha,
        float size)
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.z = size;
    marker.pose.orientation.w = 1.0f;
    marker.text = string_label;
    return marker;
}

std::string Utils::getMatrixAsString(
        const std::vector<float>& mat,
        size_t N)
{
    std::ostringstream mat_stream;
    assert(N*N == mat.size());
    for ( size_t i = 0; i < N; i++ )
    {
        for ( size_t j = 0; j < N; j++ )
        {
            mat_stream << Utils::roundFloat(mat[i*N + j], 3) << "\t";
        }
        mat_stream << std::endl;
    }
    return mat_stream.str();
}

} // namespace geometry_common

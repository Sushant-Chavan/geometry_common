#include <geometry_common/pointcloud_projector.h>
#include <math.h>

using geometry_common::Point;
using geometry_common::Utils;

PointCloudProjector::PointCloudProjector():
    passthrough_min_z_(0.0f),
    passthrough_max_z_(2.0f),
    radial_dist_min_(0.1f),
    radial_dist_max_(1000.0f),
    angle_min_(-3.14f),
    angle_max_(3.14f),
    is_angle_flipped_(false),
    angle_increment_(0.01f),
    num_of_scan_pts_(0),
    external_validity_func_(NULL)
{
    /* initialise identity transformation matrix */
    camera_to_target_tf_mat_ = Utils::getTransformMat(0, 0, 0, 0, 0, 0);
}

PointCloudProjector::~PointCloudProjector()
{
}

void PointCloudProjector::configureTransform(
        float cam_x,
        float cam_y,
        float cam_z,
        float cam_roll,
        float cam_pitch,
        float cam_yaw)
{
    camera_to_target_tf_mat_ = Utils::getTransformMat(
            cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw);
}

bool PointCloudProjector::configure(
        float cam_x,
        float cam_y,
        float cam_z,
        float cam_roll,
        float cam_pitch,
        float cam_yaw,
        float passthrough_min_z,
        float passthrough_max_z,
        float radial_dist_min,
        float radial_dist_max,
        float angle_min,
        float angle_max,
        float angle_increment)
{
    configureTransform(cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw);

    passthrough_min_z_ = passthrough_min_z;
    passthrough_max_z_ = passthrough_max_z;
    radial_dist_min_ = radial_dist_min;
    radial_dist_max_ = radial_dist_max;
    angle_min_ = angle_min;
    angle_max_ = angle_max;
    is_angle_flipped_ = ( angle_min_ > angle_max_ );
    angle_increment_ = angle_increment;
    angle_increment_inv_ = 1.0f/angle_increment_;

    num_of_scan_pts_ = 1 + ceil((angle_max_ - angle_min_) * angle_increment_inv_);
    if ( is_angle_flipped_ )
    {
        num_of_scan_pts_ = ceil(((M_PI - angle_min_) + (angle_max_ + M_PI)) * angle_increment_inv_);
    }
    return true;
}

bool PointCloudProjector::configure(
        const YAML::Node& config_params_yaml)
{
    if ( !config_params_yaml.IsMap() )
    {
        std::cout << std::endl << std::endl;
        std::cerr << "Config file does not have a correct format." << std::endl;
        std::cout << std::endl << std::endl;
        return false;
    }

    float x = 0.0f, y = 0.0f, z = 0.0f, roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    if ( config_params_yaml["transform"] )
    {
        const YAML::Node& transform_yaml = config_params_yaml["transform"];
        if ( !transform_yaml.IsMap() )
        {
            std::cout << std::endl << std::endl;
            std::cerr << "Transform in config file does not have a correct format." << std::endl;
            std::cout << std::endl << std::endl;
            return false;
        }

        x     = ( transform_yaml["x"] )     ? transform_yaml["x"].as<float>()     : 0.0f;
        y     = ( transform_yaml["y"] )     ? transform_yaml["y"].as<float>()     : 0.0f;
        z     = ( transform_yaml["z"] )     ? transform_yaml["z"].as<float>()     : 0.0f;
        roll  = ( transform_yaml["roll"] )  ? transform_yaml["roll"].as<float>()  : 0.0f;
        pitch = ( transform_yaml["pitch"] ) ? transform_yaml["pitch"].as<float>() : 0.0f;
        yaw   = ( transform_yaml["yaw"] )   ? transform_yaml["yaw"].as<float>()   : 0.0f;
    }

    float passthrough_min_z = ( config_params_yaml["passthrough_min_z"] )
                              ? config_params_yaml["passthrough_min_z"].as<float>()
                              : passthrough_min_z_;
    float passthrough_max_z = ( config_params_yaml["passthrough_max_z"] )
                              ? config_params_yaml["passthrough_max_z"].as<float>()
                              : passthrough_max_z_;
    float radial_dist_min = ( config_params_yaml["radial_dist_min"] )
                            ? config_params_yaml["radial_dist_min"].as<float>()
                            : radial_dist_min_;
    float radial_dist_max = ( config_params_yaml["radial_dist_max"] )
                            ? config_params_yaml["radial_dist_max"].as<float>()
                            : radial_dist_max_;
    float angle_min = ( config_params_yaml["angle_min"] )
                      ? config_params_yaml["angle_min"].as<float>()
                      : angle_min_;
    float angle_max = ( config_params_yaml["angle_max"] )
                      ? config_params_yaml["angle_max"].as<float>()
                      : angle_max_;
    float angle_increment = ( config_params_yaml["angle_increment"] )
                            ? config_params_yaml["angle_increment"].as<float>()
                            : angle_increment_;

    return configure(x, y, z, roll, pitch, yaw, passthrough_min_z, passthrough_max_z,
                     radial_dist_min, radial_dist_max, angle_min, angle_max, angle_increment);
}

bool PointCloudProjector::configure(
        const std::string& config_file)
{
    YAML::Node config_params_yaml;
    try
    {
        config_params_yaml = YAML::LoadFile(config_file);
    }
    catch(YAML::BadFile)
    {
        std::cout << std::endl << std::endl;
        std::cerr << "YAML threw BadFile exception. Does the file exist?" << std::endl;
        std::cout << config_file << std::endl;
        std::cout << std::endl << std::endl;
        return false;
    }
    return configure(config_params_yaml);
}

void PointCloudProjector::transformPointCloud(
        std::vector<Point>& pc) const
{
    for ( Point& p : pc )
    {
        Utils::transformPoint(camera_to_target_tf_mat_, p);
    }
}

void PointCloudProjector::applyFilter(
        std::vector<Point>& pc) const
{
    std::vector<Point> result_pc;
    result_pc.reserve(pc.size());
    for ( const Point& pt : pc )
    {
        float dist = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
        float angle = atan2(pt.y, pt.x);
        if ( isPointValid(pt, dist, angle) )
        {
            result_pc.push_back(pt);
        }
    }
    pc = result_pc;
    pc.shrink_to_fit();
}

std::vector<float> PointCloudProjector::pointCloudToScan(
        const std::vector<Point>& pc) const
{
    std::vector<float> scan(num_of_scan_pts_, radial_dist_max_);

    for ( Point pt : pc )
    {
        /* tranform point into target frame */
        Utils::transformPoint(camera_to_target_tf_mat_, pt);

        /* filter point */
        float dist = pt.magnitude();
        float angle = atan2(pt.y, pt.x);
        if ( !isPointValid(pt, dist, angle) )
        {
            continue;
        }

        /* convert to scan */
        if ( is_angle_flipped_ && angle < angle_max_ && angle > -M_PI )
        {
            angle += 2*M_PI;
        }
        size_t scan_index = ((angle - angle_min_) * angle_increment_inv_) + 0.5f;
        if ( scan_index < num_of_scan_pts_ )
        {
            scan[scan_index] = std::min(dist, scan[scan_index]);
        }
    }
    return scan;
}

std::vector<Point> PointCloudProjector::pointCloudTo2DPointCloud(
        const std::vector<Point>& pc) const
{
    std::vector<float> scan = pointCloudToScan(pc);

    /* convert from scan to flat pointcloud */
    std::vector<Point> flat_pc;
    flat_pc.reserve(scan.size());
    for ( size_t i = 0; i < scan.size(); i++ )
    {
        if ( scan[i] == radial_dist_max_ )
        {
            continue;
        }
        flat_pc.push_back(Point(scan[i] * cos(angle_min_ + i*angle_increment_),
                                scan[i] * sin(angle_min_ + i*angle_increment_)));
    }
    return flat_pc;
}

bool PointCloudProjector::isPointValid(
        const Point& pt,
        float dist,
        float angle) const
{
    if ( std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z) )
    {
        return false;
    }

    /* passthrough z filter */
    if ( pt.z < passthrough_min_z_ || pt.z > passthrough_max_z_ )
    {
        return false;
    }

    /* radial dist filter */
    if ( dist < radial_dist_min_ || dist > radial_dist_max_ )
    {
        return false;
    }

    /* radial angle filter */
    if ( is_angle_flipped_ && ( angle < angle_min_ && angle > angle_max_ ) )
    {
        return false;
    }
    if ( !is_angle_flipped_ && ( angle < angle_min_ || angle > angle_max_ ) )
    {
        return false;
    }

    /* check external validity function */
    if ( external_validity_func_ != nullptr && !external_validity_func_(pt) )
    {
        return false;
    }

    return true;
}

float PointCloudProjector::getRadialDistMax() const
{
    return radial_dist_max_;
}

float PointCloudProjector::getRadialDistMin() const
{
    return radial_dist_min_;
}

void PointCloudProjector::setValidityFunction(
        ValidityFunction vf)
{
    external_validity_func_ = vf;
}

void PointCloudProjector::setPassthroughMinZ(
        float passthrough_min_z)
{
    passthrough_min_z_ = passthrough_min_z;
}

void PointCloudProjector::setPassthroughMaxZ(
        float passthrough_max_z)
{
    passthrough_max_z_ = passthrough_max_z;
}

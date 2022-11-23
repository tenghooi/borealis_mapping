#include "ray_caster.h"

RayCaster::RayCaster()
{

}

RayCaster::~RayCaster() {}

void RayCaster::set_ray_origin(const Eigen::Vector3d& origin_point)
{
    ray_origin_ = origin_point;
}

void RayCaster::set_ray_origin(const double& x, const double& y, const double& z)
{
    Eigen::Vector3d origin_point(x, y, z);

    set_ray_origin(origin_point);
}

void RayCaster::set_ray_end(const Eigen::Vector3d& end_point)
{
    ray_end_ = end_point;
}

void RayCaster::set_ray_end(const double& x, const double& y, const double& z)
{
    Eigen::Vector3d end_point(x, y, z);

    set_ray_end(end_point);
}

void RayCaster::set_end_points(const Eigen::Vector3d& origin_point, const Eigen::Vector3d& end_point)
{
    set_ray_origin(origin_point);
    set_ray_end(end_point);
}

void RayCaster::RayCasting2D()
{

}

void RayCaster::RayCasting3D()
{

}

void RayCaster::RayCastingBresenham2D()
{

}

void RayCaster::RayCastingBresenham3D()
{
    
}
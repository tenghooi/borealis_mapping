#include "ray_caster.h"

RayCaster::RayCaster()
{
    ray_origin_ << 0.0, 0.0, 0.0;
    ray_end_ << 0.0, 0.0, 0.0;
}

RayCaster::~RayCaster() {}

Eigen::Vector3d RayCaster::get_ray_origin()
{
    return ray_origin_;
}

Eigen::Vector3d RayCaster::get_ray_end()
{
    return ray_end_;
}

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

void RayCaster::RayCasting2D(std::vector<Eigen::Vector2d>& traversed_voxels)
{
// Initialization Step
    Eigen::Vector2d current_voxel;
    current_voxel.x() = std::floor(ray_origin_.x());
    current_voxel.y() = std::floor(ray_origin_.y());

    Eigen::Vector2d ray_direction;
    ray_direction.x() = ray_end_.x() - ray_origin_.x();
    ray_direction.y() = ray_end_.y() - ray_origin_.y();

    double dx = ray_direction.x();
    double dy = ray_direction.y(); 
    
    // step* indicates wheter X or Y are incremented or decremented as the ray crosses voxel boundaries.
    int stepX = (dx == 0) ? 0 : (dx > 0) ? 1 : -1;
    int stepY = (dy == 0) ? 0 : (dy > 0) ? 1 : -1;

    // t breaks down the ray into intervals of t such that each interval spans one voxel.
    // t is the value which ray first crosses the vertical(horizontal) boundary inside the grid.
    // t is incremental from initial t where ray crossed the first boundary to 1 where 
    // it reaches ray end.
    //
    // tMax* >= 0 and tMax* <= 1. When tMax* > 1, it exceeds the ray length.
    // r = x0 + tMaxX * dx, where r is the x of the first crossed vertical boundary.
    // Hence, tMaxX = (r - x0) / dx       OR
    //        tMaxX = (current_voxel_x + voxel_width - x0) / dx for positive direction.
    // If dx = 0, that means ray will never travel horizontally. Set tMaxX to be large number(1000).

    double tMaxX = (dx > 0) ? (current_voxel.x() + stepX - ray_origin_.x()) / dx : \
                   (dx < 0) ? (current_voxel.x() - ray_origin_.x()) / dx : 1000;
    double tMaxY = (dy > 0) ? (current_voxel.y() + stepY - ray_origin_.y()) / dy : \
                   (dy < 0) ? (current_voxel.y() - ray_origin_.y()) / dy : 1000;
    
    // tDeltaX indicates how far along the ray we must move (in units of t) for the horizontal 
    // component of such a movement to equal the width of a voxel.
    // In other words, divide the width of a voxel by dX or dY of the ray.
    // E.g. Voxel size = 1; dX = 5.0;
    //      Then, tDeltaX = 1 / 5;
    //
    // tDelta* must be positive like tMax*, i.e. tDelta* >= 0 and <= 1

    double tDeltaX = (dx != 0) ? static_cast<double>(stepX) / dx : 1000;
    double tDeltaY = (dy != 0) ? static_cast<double>(stepY) / dy : 1000;

    traversed_voxels.clear();

    // Add the voxel that contains ray origin to the traversed_voxels vector.
    traversed_voxels.push_back(current_voxel);

// Traversal Step
    while (tMaxX < 1 || tMaxY < 1) // 1 or >1 means reached end of ray.
    {
        if (tMaxX < tMaxY)
        {
            tMaxX = tMaxX + tDeltaX;
            current_voxel.x() = current_voxel.x() + stepX;
        }
        else
        {
            tMaxY = tMaxY +tDeltaY;
            current_voxel.y() = current_voxel.y() + stepY;
        }

        traversed_voxels.push_back(current_voxel);
    }
}

void RayCaster::RayCasting3D(std::vector<Eigen::Vector3d>& traversed_voxels)
{
// Initialization Step
    Eigen::Vector3d current_voxel;
    current_voxel.x() = std::floor(ray_origin_.x());
    current_voxel.y() = std::floor(ray_origin_.y());
    current_voxel.z() = std::floor(ray_origin_.z());

    Eigen::Vector3d ray_direction;
    ray_direction = ray_end_ - ray_origin_;

    double dx = ray_direction.x();
    double dy = ray_direction.y();
    double dz = ray_direction.z();

    int stepX = (dx == 0) ? 0 : (dx > 0) ? 1 : -1;
    int stepY = (dy == 0) ? 0 : (dy > 0) ? 1 : -1;
    int stepZ = (dz == 0) ? 0 : (dz > 0) ? 1 : -1;

    double tMaxX = (dx > 0) ? (current_voxel.x() + stepX - ray_origin_.x()) / dx : \
                   (dx < 0) ? (current_voxel.x() - ray_origin_.x()) / dx : 1000;
    double tMaxY = (dy > 0) ? (current_voxel.y() + stepY - ray_origin_.y()) / dy : \
                   (dy < 0) ? (current_voxel.y() - ray_origin_.y()) / dy : 1000;
    double tMaxZ = (dz > 0) ? (current_voxel.z() + stepZ - ray_origin_.z()) / dz : \
                   (dz < 0) ? (current_voxel.z() - ray_origin_.z()) / dz : 1000;
    
    double tDeltaX = (dx != 0) ? static_cast<double>(stepX) / dx : 1000;
    double tDeltaY = (dy != 0) ? static_cast<double>(stepY) / dy : 1000;
    double tDeltaZ = (dz != 0) ? static_cast<double>(stepZ) / dz : 1000;

    traversed_voxels.push_back(current_voxel);

// Traversal Step
    while (tMaxX < 1 || tMaxY < 1 || tMaxZ < 1)
    {
        if (tMaxX < tMaxY)
        {
            if (tMaxX < tMaxZ)
            {
                tMaxX = tMaxX + tDeltaX;
                current_voxel.x() = current_voxel.x() + stepX;
            }
            else
            {
                tMaxZ = tMaxZ + tDeltaZ;
                current_voxel.z() = current_voxel.z() + stepZ;
            }
        }
        else
        {
            if (tMaxY < tMaxZ)
            {
                tMaxY = tMaxY + tDeltaY;
                current_voxel.y() = current_voxel.y() + stepY;
            }
            else
            {
                tMaxZ = tMaxZ + tDeltaZ;
                current_voxel.z() = current_voxel.z() + stepZ;
            }

        }

        traversed_voxels.push_back(current_voxel);
    }
}

void RayCaster::RayCastingBresenham2D()
{
    //TODO
}

void RayCaster::RayCastingBresenham3D()
{
    //TODO
}
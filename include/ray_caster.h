#ifndef _RAY_CASTER_H_
#define _RAY_CASTER_H_

#include <vector>
#include <iostream>
#include <cmath>

#include <Eigen/Core>

class RayCaster
{
private:
    Eigen::Vector3d ray_origin_;
    Eigen::Vector3d ray_end_;

public:
    RayCaster();
    ~RayCaster();

    void set_ray_origin(const Eigen::Vector3d& origin_point);
    void set_ray_origin(const double& x, const double& y, const double& z);
    void set_ray_end(const Eigen::Vector3d& end_point);
    void set_ray_end(const double& x, const double& y, const double& z);

    void set_end_points(const Eigen::Vector3d& origin_point, const Eigen::Vector3d& end_point);

/*
    RayCasting2D and RayCasting3D are based on Amanatides & Woo ray tracing algorithm.
    Their paper "A Fast Voxel Traversal Algorithm for Ray Tracing" can be found here: 
    <https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.42.3443&rep=rep1&type=pdf>
*/
    void RayCasting2D();
    void RayCasting3D();

    void RayCastingBresenham2D();
    void RayCastingBresenham3D();

};




#endif // _RAY_CASTER_H_


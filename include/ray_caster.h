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

    void set_ray_origin(Eigen::Vector3d& ray_origin);
    void set_ray_origin(double& x, double& y, double& z);
    void set_ray_end(Eigen::Vector3d& ray_end);
    void set_ray_end(double& x, double& y, double& z);

    void set_end_points(Eigen::Vector3d& ray_origin, Eigen::Vector3d& ray_end);

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


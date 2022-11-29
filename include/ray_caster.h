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

    This algorithm uses a parameterized line equation for ray tracing.
    Ray equation:  r = r0 + t*v, where r0 is the ray origin, v is difference between ray end and ray origin.
    
    E.g: X0 = [0.5 0.5], X1 = [2.5 0]
         Then, v = [2 -0.5] 
         r = [0.5 0.5] + t * [2 -0.5]
*/
    void RayCasting2D(std::vector<Eigen::Vector2d>& traversed_voxels);
    void RayCasting3D();

    void RayCastingBresenham2D();
    void RayCastingBresenham3D();

};




#endif // _RAY_CASTER_H_


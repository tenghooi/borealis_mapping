#ifndef _RAY_CASTER_H_
#define _RAY_CASTER_H_

#include <vector>
#include <iostream>
#include <cmath>

#include <Eigen/Core>

class RayCaster
{
private:


public:
    RayCaster();
    ~RayCaster();

    void RayCasting2D();
    void RayCasting3D();

    void RayCastingBresenham2D();
    void RayCastingBresenham3D();

};




#endif // _RAY_CASTER_H_


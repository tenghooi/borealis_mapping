#ifndef _BOREALIS_MAPPING_DYNAMIC_OBJECT_
#define _BOREALIS_MAPPING_DYNAMIC_OBJECT_

#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Eigen>

#include <ros/ros.h>

class DynamicObject
{
private:
    //Bounding box size vectors for object
    Eigen::Vector4f box_max_vec_;
    Eigen::Vector4f box_min_vec_;



public:

    DynamicObject();
    ~DynamicObject();

    Eigen::Vector4f GetMaxVec() const;
    Eigen::Vector4f GetMinVec() const;

    void SetMinMaxVec();
    


};

#endif //_BOREALIS_MAPPING_DYNAMIC_OBJECT_
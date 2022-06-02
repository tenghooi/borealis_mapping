#include "object_filter.h"

int main(int argc, char** argv)
{           
    ros::init(argc, argv, "borealis_object_filter");
    ros::NodeHandle node("~");
    
    ObjectsFiltering objects_filter(node);
    
    ros::spin();
    return 0;
}
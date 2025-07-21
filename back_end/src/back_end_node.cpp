#include "back_end/bspline_opt.h"
#include <ros/ros.h>

int main( int argc, char * argv[] )
{ 
    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh("~");
    
    ros::spin();

    return 0;
}
#include "front_end/theta_astar.h"
#include <ros/ros.h>

int main( int argc, char * argv[] )
{ 
    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh("~");
    
    ros::spin();

    return 0;
}
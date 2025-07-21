#include <ros/ros.h>
#include "plan_manager/replan_fsm.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "manager_node");
    ros::NodeHandle nh("~");

    ReplanFsm replan_fsm_;
    replan_fsm_.init(nh);
    replan_fsm_._fsm_sign = 0;
    
    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}

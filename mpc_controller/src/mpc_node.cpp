#include "mpc_controller/mpc_controller.h"
#include <ros/ros.h>

int main( int argc, char * argv[] )
{ 
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh;

  MPC_PLANNER mpc_planner_;
  mpc_planner_.init(nh);

  ros::Duration(1.0).sleep();

  ros::spin();

  return 0;
}
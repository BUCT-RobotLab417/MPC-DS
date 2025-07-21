#pragma once

// cpp 头文件
#include <fstream>
#include <string.h>
#include <random>
#include <time.h>

// ros头文件
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>


// include planner
#include "plan_env/edt_environment.h"
#include "front_end/theta_astar.h"
#include "back_end/bspline_opt.h"


class PlanManager
{
    private:
        /*-------ESDF 局部地图信息---------*/ 
        SDFMap::Ptr sdf_map_;
        fast_planner::EDTEnvironment::Ptr edt_environment_;   // ESDF地图

        /*-------global planner信息---------*/ 
        std::unique_ptr<Bspline_Opt> bspline_optimizers_;     
        std::unique_ptr<ThetaAstar> theta_astar_finder_;

        /*--------参数----------*/
        double sample_time_; // 采样时间
        int status_;                            // 路径搜索状态
        std::vector<Eigen::Vector2d> trajlist_; // 存储轨迹
        

    public:
        void init(ros::NodeHandle& nh);
        bool hybridReplanFsm(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel, Eigen::Vector2d start_acc, Eigen::Vector2d end_pt, Eigen::Vector2d end_vel, double start_yaw);

        std::vector<Eigen::Vector2d> getGlobalPath(double step_time);
        std::vector<ros::Time> get_time_list(double step_time);
        Eigen::Vector2d evaluateFrontPose(ros::Time& t, std::vector<ros::Time>& tlist);

        typedef shared_ptr<PlanManager> Ptr;
};
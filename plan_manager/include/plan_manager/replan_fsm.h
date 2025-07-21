#pragma once

#include "plan_manager/plan_manager.h"


class ReplanFsm
{
private:

    std::unique_ptr<PlanManager> plan_manager_;

    /*----ros接口----*/
    ros::Subscriber odom_sub;   // 接收里程计信息
    ros::Subscriber target_sub; // 接收目标点
    ros::Publisher mpc_traj_pub_, simpleMpc_traj_pub_; // 发送轨迹信息
    ros::Timer mainte_fsm, globbal_path_timer;

    /*--------定义全局变量----------*/
    Eigen::Vector3d odom_pos; // 机器人的位置信息[x,y,yaw]
    bool is_odom_rcv_, is_target_rcv_;
    double start_yaw_, cur_yaw_;               // 记录yaw规划的起点
    Eigen::Vector2d start_pt_, start_vel_, start_acc_;  // start state
    Eigen::Vector2d cur_pt_, cur_vel_;  // start state
    Eigen::Vector2d end_pt_, end_vel_;                  // target state
    bool plan_scuuess;
    int status_; // 0:无目标 1:规划中 2:规划成功 3:规划失败
    // 状态机标志位0-init, 1-wait_target, 2-gen_new_traj, 3-exec_traj, 4-replan_traj
    double v_max;   // 机器人的最大速度
    double step_time;
    bool static_path;
    double replan_thresh1, replan_thresh2;

    /*----轨迹信息----------*/
    std::vector<Eigen::Vector2d> trajlist_;
    std::vector<ros::Time> timelist_;
    ros::Time start_time_, end_time_;


    /*----helper-----*/
    double quaternion2Yaw(geometry_msgs::Quaternion q); // 四原数转yaw角    

    /*----CallBack-----*/
    void rcvOdomCallBack(const nav_msgs::OdometryConstPtr& msg);
    void rcvWpsCallBack(const geometry_msgs::PoseStamped msg);
    void maintenance_fsm(const ros::TimerEvent& e);
    void globalPathPub_Callback(const ros::TimerEvent& e);
    
public:
    void init(ros::NodeHandle& nh);
    int _fsm_sign;
};
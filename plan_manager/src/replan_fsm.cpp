#include "plan_manager/replan_fsm.h"

void ReplanFsm::init(ros::NodeHandle& nh)
{
    // 初始化参数
    nh.param("search/max_vel", v_max, 1.0);
    nh.param("replan_fsm/step_time", step_time, 1.0);
    nh.param("replan_fsm/thresh_replan", replan_thresh1, -1.0);
    nh.param("replan_fsm/thresh_no_replan", replan_thresh2, -1.0);
    // nh.param("replan_fsm/static_path", static_path, false);


    // 初始化plan_manager
    plan_manager_.reset(new PlanManager);
    plan_manager_->init(nh);   

    // ros接口
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 50, &ReplanFsm::rcvOdomCallBack, this);
    target_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &ReplanFsm::rcvWpsCallBack, this);

    mpc_traj_pub_ = nh.advertise<nav_msgs::Path>("/global_path", 20);
    simpleMpc_traj_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/simpleMpc_global_path", 20);

    mainte_fsm = nh.createTimer(ros::Duration(0.02), &ReplanFsm::maintenance_fsm, this);
    globbal_path_timer = nh.createTimer(ros::Duration(0.1), &ReplanFsm::globalPathPub_Callback, this);

    // 初始化标志位
    is_odom_rcv_ = false;
    is_target_rcv_ = false;

    ROS_INFO("ReplanFsm Init success");
    return;
}

double ReplanFsm::quaternion2Yaw(geometry_msgs::Quaternion q)
{
    tf::Quaternion tfq(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 m(tfq);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void ReplanFsm::rcvOdomCallBack(const nav_msgs::OdometryConstPtr& msg)
{
    cur_pt_ << msg->pose.pose.position.x, msg->pose.pose.position.y;
    cur_yaw_ = quaternion2Yaw(msg->pose.pose.orientation);
    // 根据yaw角给定一个初始速度-用于路径搜索
    double init_vel = 1.2;
    cur_vel_ << init_vel * cos(start_yaw_), init_vel * sin(start_yaw_);    
    start_acc_.setZero();

    is_odom_rcv_ = true;
}

void ReplanFsm::rcvWpsCallBack(const geometry_msgs::PoseStamped msg)
{
    end_pt_ << msg.pose.position.x, msg.pose.position.y;
    end_vel_.setZero();

    is_target_rcv_ = true;
}

void ReplanFsm::maintenance_fsm(const ros::TimerEvent& e)
{
    // ROS_INFO("-------ReplanFsm maintenance_fsm-----------");
    static int fsm_num = 0;
    fsm_num += 1;
    if (fsm_num==40){
        if(!is_odom_rcv_)   std::cout << "............ no odom ............." << std::endl;
        if(!is_target_rcv_) std::cout << "............ wait for goal ............." << std::endl;
    }
    switch (_fsm_sign)
    {
        case 0: // 判断是否接收到里程计信息
        {
            if(!is_odom_rcv_) return;
            /* change fsm from 0 to 1 */
            _fsm_sign = 1;
            break;
        }
        case 1: // 判断是否接收到目标点
        {
            if(!is_target_rcv_) return;
            else{
                // 当前位置与终点小于设定阈值时，停止规划
                Eigen::Vector2d target_pt, odom_now;
                target_pt   << end_pt_[0], end_pt_[1];
                odom_now    << cur_pt_[0], cur_pt_[1];
                if((target_pt - odom_now).norm() < 0.4) {
                    is_target_rcv_ = false;
                    break;
                }
                else{
                    _fsm_sign = 2;
                }
                break;
            }            
        }
        // 运动规划
        case 2:
        {
            start_pt_ = cur_pt_;
            double init_vel = v_max * 0.8;
            start_vel_<< init_vel * cos(cur_yaw_), init_vel * sin(cur_yaw_);
            start_acc_.setZero();
            start_yaw_ = cur_yaw_;
            // 初始化路径搜索
            plan_scuuess = plan_manager_->hybridReplanFsm(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, start_yaw_);
            if(plan_scuuess){
                trajlist_ = plan_manager_->getGlobalPath(step_time);
                timelist_ = plan_manager_->get_time_list(step_time);
                start_time_ = timelist_.front();
                end_time_   = timelist_.back();
                _fsm_sign = 3;
            }
            break;
        }
        // 运动执行
        case 3:
        {
            ros::Time t_cur = ros::Time::now();
            Eigen::Vector2d pos = plan_manager_->evaluateFrontPose(t_cur, timelist_);
            if((t_cur-end_time_).toSec() >-1e-2){
                is_target_rcv_ = false;
                /* change fsm from 3 to 1 */
                _fsm_sign = 1;
            }
            else if((end_pt_-pos).norm()<replan_thresh1){ 
                return;
            }
            else if((start_pt_-pos).norm()<replan_thresh2) {
                return;
            }
            else{
                _fsm_sign = 4;
            }
        }
        // 轨迹重规划
        case 4:
        {
            std::cout<< "\033[33m............ replan_traj .............\033[0m\n";
            start_pt_ = cur_pt_;
            double init_vel = v_max;
            start_vel_<< init_vel * cos(cur_yaw_), init_vel * sin(cur_yaw_);
            start_acc_.setZero();
            start_yaw_ = cur_yaw_;
            plan_scuuess = plan_manager_->hybridReplanFsm(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, start_yaw_);     
            if(plan_scuuess){
                trajlist_ = plan_manager_->getGlobalPath(step_time);
                timelist_ = plan_manager_->get_time_list(step_time);
                start_time_ = timelist_.front();
                end_time_   = timelist_.back();
                _fsm_sign = 3;
            }
            else{
                _fsm_sign = 2;
            }
            break;
        }
    }
}

void ReplanFsm::globalPathPub_Callback(const ros::TimerEvent& e)
{
    int num = static_cast<int>(trajlist_.size());
    if (num == 0) return;
    nav_msgs::Path global_path;
    global_path.header.stamp = ros::Time::now();
    global_path.header.frame_id = "odom";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "odom";
    pose_stamped.pose.orientation.x = 0;
    pose_stamped.pose.orientation.y = 0;
    pose_stamped.pose.orientation.z = 0;
    pose_stamped.pose.orientation.w = 1;

    int idx = 0;
    for(int i=0; i<num; i++){
        pose_stamped.header.seq = idx++;
        pose_stamped.pose.position.x = trajlist_[i].x();
        pose_stamped.pose.position.y = trajlist_[i].y();
        pose_stamped.pose.position.z = 0.0;

        global_path.poses.push_back(pose_stamped);
    }
    mpc_traj_pub_.publish(global_path);
}


#include "plan_manager/plan_manager.h"


void PlanManager::init(ros::NodeHandle& nh)
{
    // ESDF初始化
    sdf_map_.reset(new SDFMap);
    sdf_map_->initMap(nh);
    edt_environment_.reset(new fast_planner::EDTEnvironment);
    edt_environment_->setMap(sdf_map_);

    // path_finder
    theta_astar_finder_.reset(new ThetaAstar);
    theta_astar_finder_->init(nh, edt_environment_);

    ROS_INFO("Plan manager init success");
    return;
}


bool PlanManager::hybridReplanFsm(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel, Eigen::Vector2d start_acc,
                               Eigen::Vector2d end_pt, Eigen::Vector2d end_vel, double start_yaw)
{
    clock_t search_time_start, search_time_end;    //定义clock_t变量用于计时
    search_time_start = clock();           //开始时间

    theta_astar_finder_->reset();
    status_ = theta_astar_finder_->search(start_pt, start_vel, start_acc, start_yaw, end_pt, end_vel, false, true, -1.0); // 带theta的kino astar
    if(status_ == NO_PATH){
        theta_astar_finder_->reset();
        status_ = theta_astar_finder_->search(start_pt, start_vel, start_acc, start_yaw, end_pt, end_vel, false, true, -1.0);
        if(status_ == NO_PATH){
            ROS_WARN("No path found!");
            return false;
        }
    }

    search_time_end = clock();   // 结束时间
    double search_time_spend = double(search_time_end - search_time_start) / CLOCKS_PER_SEC;
    std::cout<< "[PlanManager replan]: Kino spend time:." <<search_time_spend << std::endl;
    return true;
}

std::vector<Eigen::Vector2d> PlanManager::getGlobalPath(double step_time)
{
    std::vector<Eigen::Vector2d> trajlist;
    trajlist = theta_astar_finder_->getKinoTraj(step_time);
    // for (const auto& point : trajlist){
    //     std::cout << "x: " << point.x() << ", y: " << point.y() << std::endl;
    // }
    return trajlist;
}

std::vector<ros::Time> PlanManager::get_time_list(double step_time)
{
    std::vector<ros::Time> timelist;
    timelist = theta_astar_finder_->getKinoTraj_t(step_time);
    return timelist; 
}

Eigen::Vector2d PlanManager::evaluateFrontPose(ros::Time& t, std::vector<ros::Time>& tlist)
{
    int closeIndex;
    std::vector<Eigen::Vector2d> traj_list;
    Eigen::Vector2d evaluPos;
    double minDiff = std::numeric_limits<double>::max();

    for(int i=0; i<tlist.size(); i++){
        double diff = std::fabs(t.toSec() - tlist[i].toSec());
        if(diff<minDiff){
        minDiff     = diff;
        closeIndex  = static_cast<int>(i);
        }
    }
    traj_list = theta_astar_finder_->getKinoTraj(0.2);
    evaluPos = traj_list[closeIndex];
    // std::cout<< closeIndex<< std::endl;
    // std::cout<< "............ here .............\n";
    // std::cout<< evaluPos<< std::endl;
    return evaluPos;
}
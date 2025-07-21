#include <nlopt.hpp>
#include "back_end/bspline_opt.h"

// 配置优化条件
const int Bspline_Opt::SMOOTHNESS       = (1 << 0);
const int Bspline_Opt::DISTANCE         = (1 << 1);
const int Bspline_Opt::FEASIBILITY      = (1 << 2);
const int Bspline_Opt::ENDPOINT         = (1 << 3);
const int Bspline_Opt::GUIDE            = (1 << 4);
const int Bspline_Opt::WAYPOINTS        = (1 << 6);
const int Bspline_Opt::CURVATURE        = (1 << 7);
const int Bspline_Opt::VELOCITYOBSTACLE = (1 << 8);

const int Bspline_Opt::GUIDE_PHASE = Bspline_Opt::SMOOTHNESS | Bspline_Opt::GUIDE;
const int Bspline_Opt::NORMAL_PHASE = Bspline_Opt::SMOOTHNESS | Bspline_Opt::DISTANCE | Bspline_Opt::FEASIBILITY | Bspline_Opt::VELOCITYOBSTACLE;


void Bspline_Opt::init(ros::NodeHandle& nh, const fast_planner::EDTEnvironment::Ptr& env) 
{
  setParam(nh);
  setEnvironment(env);
}

void Bspline_Opt::setParam(ros::NodeHandle& nh) {
    nh.param("optimization/lambda_Smoothness",  lambda_Smoothness_,   -1.0);
    nh.param("optimization/lambda_SdfDistance", lambda_SdfDistance_,  -1.0);
    nh.param("optimization/lambda_Feasibility", lambda_Feasibility_,  -1.0);
    nh.param("optimization/lambda_EndPoint",    lambda_EndPoint_,     -1.0);
    nh.param("optimization/lambda_GuidePoint",  lambda_GuidePoint_,   -1.0);
    nh.param("optimization/lambda_WayPoint",    lambda_WayPoint_,     -1.0);
    nh.param("optimization/lambda_Curvature",   lambda_Curvature_,    -1.0);

    nh.param("optimization/dist0", dist0_, -1.0);
    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);
    nh.param("optimization/visib_min", visib_min_, -1.0);
    nh.param("optimization/dlmin", dlmin_, -1.0);
    nh.param("optimization/wnl", wnl_, -1.0);

    nh.param("optimization/max_iteration_num1", max_iteration_num_[0], -1);
    nh.param("optimization/max_iteration_num2", max_iteration_num_[1], -1);
    nh.param("optimization/max_iteration_num3", max_iteration_num_[2], -1);
    nh.param("optimization/max_iteration_num4", max_iteration_num_[3], -1);
    nh.param("optimization/max_iteration_time1", max_iteration_time_[0], -1.0);
    nh.param("optimization/max_iteration_time2", max_iteration_time_[1], -1.0);
    nh.param("optimization/max_iteration_time3", max_iteration_time_[2], -1.0);
    nh.param("optimization/max_iteration_time4", max_iteration_time_[3], -1.0);

    nh.param("optimization/algorithm1", algorithm1_, -1);
    nh.param("optimization/algorithm2", algorithm2_, -1);
    nh.param("optimization/order", order_, 3);
}

void Bspline_Opt::setEnvironment(const fast_planner::EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
}

Eigen::MatrixXd Bspline_Opt::BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                                      const int& cost_function, int max_num_id,
                                                      int max_time_id) {
  setControlPoints(points);
  setBsplineInterval(ts);
  setCostFunction(cost_function);
  setTerminateCond(max_num_id, max_time_id);
  setStartTime(ros::Time::now());
  optimize();
  return this->control_points_;
}


double Bspline_Opt::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data) {
  Bspline_Opt* opt = reinterpret_cast<Bspline_Opt*>(func_data);
  double            cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_      = cost;
    opt->best_variable_ = x;
  }
  return cost;
}

void Bspline_Opt::combineCost(const std::vector<double>& x, std::vector<double>& grad,
                                   double& f_combine) {
  /* convert the NLopt format vector to control points. */

  // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store each control point
  // For 1D case, the second and third elements are zero, and similar for the 2D case.
  for (int i = 0; i < order_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i][j] = control_points_(i, j);
    }
  }

  for (int i = 0; i < variable_num_ / dim_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i + order_][j] = x[dim_ * i + j];
    }
  }

  if (!(cost_function_ & ENDPOINT)) {
    for (int i = 0; i < order_; i++) {
      for (int j = 0; j < dim_; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] =
            control_points_(control_points_.rows() - order_ + i, j);
      }
    }
  }

  f_combine = 0.0;
  grad.resize(variable_num_);
  std::fill(grad.begin(), grad.end(), 0.0);

  /*  evaluate costs and their gradient  */
  double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_waypoints, f_curature, f_VOcost_;
  f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = f_curature = f_VOcost_ = 0.0;

  if (cost_function_ & SMOOTHNESS) {
    calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
    f_combine += lambda_Smoothness_ * f_smoothness;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda_Smoothness_ * g_smoothness_[i + order_](j);
  }
  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += lambda_SdfDistance_ * f_distance;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda_SdfDistance_ * g_distance_[i + order_](j);
  }
  if (cost_function_ & FEASIBILITY) {
    calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
    f_combine += lambda_Feasibility_ * f_feasibility;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda_Feasibility_ * g_feasibility_[i + order_](j);
  }
  if (cost_function_ & ENDPOINT) {
    calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
    f_combine += lambda_EndPoint_ * f_endpoint;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda_EndPoint_ * g_endpoint_[i + order_](j);
  }
  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, f_guide, g_guide_);
    f_combine += lambda_GuidePoint_ * f_guide;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda_GuidePoint_ * g_guide_[i + order_](j);
  }
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += lambda_WayPoint_ * f_waypoints;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda_WayPoint_ * g_waypoints_[i + order_](j);
  }
  if (cost_function_ & CURVATURE) {
    calcCulvatureCost(g_q_, f_curature, g_culvature_);   // 加入曲率约束
    f_combine += lambda_Curvature_ * f_curature;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda_Curvature_ * g_culvature_[i + order_](j);
  }

  /*  print cost  */
  // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
  //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << lambda8_ * f_view
  //        << ", waypt: " << lambda7_ * f_waypoints << endl;
  // }

  // if (optimization_phase_ == SECOND_PHASE) {
  //  << ", smooth: " << lambda1_ * f_smoothness
  //  << " , dist:" << lambda2_ * f_distance
  //  << ", fea: " << lambda3_ * f_feasibility << endl;
  // << ", end: " << lambda4_ * f_endpoint
  // << ", guide: " << lambda5_ * f_guide
  // }                            
}

void Bspline_Opt::calcSmoothnessCost(const vector<Eigen::Vector2d>& q, double& cost,
                                          vector<Eigen::Vector2d>& gradient) {
  cost = 0.0;
  Eigen::Vector2d zero(0.0, 0.0);
  std::fill(gradient.begin(), gradient.end(), zero);
  Eigen::Vector2d jerk, temp_j;

  for (long unsigned int i = 0; i < q.size() - order_; i++) { // 最小化加加速度Jerk
    /* evaluate jerk */
    jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
    cost += jerk.squaredNorm();
    temp_j = 2.0 * jerk;
    /* jerk gradient */
    gradient[i + 0] += -temp_j;
    gradient[i + 1] += 3.0 * temp_j;
    gradient[i + 2] += -3.0 * temp_j;
    gradient[i + 3] += temp_j;
  }

  double weight_acc_ = 0.1;                 // 加速度权重
  Eigen::Vector2d acc, temp_a;
  for (long unsigned int i = 0; i < q.size() - 2; i++) {  // 最小化加速度Acc
    /* evaluate jerk */
    acc = q[i + 2] - 2 * q[i + 1] + q[i];
    cost += weight_acc_ * acc.squaredNorm();
    temp_a = 2.0 * acc;
    /* jerk gradient */
    gradient[i + 0] += weight_acc_ * temp_a;
    gradient[i + 1] += -2.0 * weight_acc_ * temp_a;
    gradient[i + 2] += weight_acc_ * temp_a;
  }
}

void Bspline_Opt::calcDistanceCost(const vector<Eigen::Vector2d>& q, double& cost,
                                        vector<Eigen::Vector2d>& gradient) {
  cost = 0.0;
  Eigen::Vector2d zero(0.0, 0.0);
  std::fill(gradient.begin(), gradient.end(), zero);

  double          dist;
  Eigen::Vector3d dist_grad;

  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

  for (int i = order_; i < end_idx; i++) {

    Eigen::Vector3d check_pt(q[i].x(), q[i].y(), 0.2);    // 取 0.2m 高度的点计算与障碍物间的距离

    edt_environment_->evaluateEDTWithGrad(check_pt, -1.0, dist, dist_grad);
    
    Eigen::Vector2d dist_grad_2d(dist_grad.x(), dist_grad.y());

    if (dist_grad_2d.norm() > 1e-4) dist_grad_2d.normalize();

    if (dist < dist0_) {
      cost += pow(dist - dist0_, 2);
      gradient[i] += 2.0 * (dist - dist0_) * dist_grad_2d;
    }
  }
}

void Bspline_Opt::calcFeasibilityCost(const vector<Eigen::Vector2d>& q, double& cost,
                                           vector<Eigen::Vector2d>& gradient) {
  cost = 0.0;
  Eigen::Vector2d zero(0.0, 0.0);
  std::fill(gradient.begin(), gradient.end(), zero);

  /* abbreviation */
  double ts, vm2, am2, ts_inv2, ts_inv4;
  vm2 = max_vel_ * max_vel_;
  am2 = max_acc_ * max_acc_;

  ts      = bspline_interval_;
  ts_inv2 = 1 / ts / ts;
  ts_inv4 = ts_inv2 * ts_inv2;

  /* velocity feasibility */
  for (long unsigned int i = 0; i < q.size() - 1; i++) {  // 第i个控制点
    Eigen::Vector2d vi = q[i + 1] - q[i];   // 速度控制点

    for (int j = 0; j < 2; j++) {   // j是xy维度
      double vd = vi(j) * vi(j) * ts_inv2 - vm2;
      if (vd > 0.0) {
        cost += pow(vd, 2);

        double temp_v = 4.0 * vd * ts_inv2;
        gradient[i + 0](j) += -temp_v * vi(j);
        gradient[i + 1](j) += temp_v * vi(j);
      }
    }
  }

  /* acceleration feasibility */
  for (long unsigned int i = 0; i < q.size() - 2; i++) {
    Eigen::Vector2d ai = q[i + 2] - 2 * q[i + 1] + q[i];  // 加速度控制点

    for (int j = 0; j < 2; j++) {
      double ad = ai(j) * ai(j) * ts_inv4 - am2;
      if (ad > 0.0) {
        cost += pow(ad, 2);

        double temp_a = 4.0 * ad * ts_inv4;
        gradient[i + 0](j) += temp_a * ai(j);
        gradient[i + 1](j) += -2 * temp_a * ai(j);
        gradient[i + 2](j) += temp_a * ai(j);
      }
    }
  }
}

void Bspline_Opt::calcEndpointCost(const vector<Eigen::Vector2d>& q, double& cost,
                                        vector<Eigen::Vector2d>& gradient) {
  cost = 0.0;
  Eigen::Vector2d zero(0.0, 0.0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // zero cost and gradient in hard constraints
  Eigen::Vector2d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
  cost += dq.squaredNorm();

  gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
  gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
}

void Bspline_Opt::calcWaypointsCost(const vector<Eigen::Vector2d>& q, double& cost,
                                         vector<Eigen::Vector2d>& gradient) {
  cost = 0.0;
  Eigen::Vector2d zero(0.0, 0.0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector2d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (long unsigned int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector2d waypt = waypoints_[i];
    int             idx   = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient[idx + 2] += dq * (2.0 / 6.0);
  }
}

void Bspline_Opt::calcGuideCost(const vector<Eigen::Vector2d>& q, double& cost,
                                vector<Eigen::Vector2d>& gradient) {
  cost = 0.0;
  Eigen::Vector2d zero(0.0, 0.0);
  std::fill(gradient.begin(), gradient.end(), zero);

  int end_idx = q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    Eigen::Vector2d gpt = guide_pts_[i - order_];
    cost += (q[i] - gpt).squaredNorm();
    gradient[i] += 2 * (q[i] - gpt);
  }
}

void Bspline_Opt::calcCulvatureCost(const vector<Eigen::Vector2d>& q, double& cost,
                                    vector<Eigen::Vector2d>& gradient) {
  cost = 0.0;   // 代价初始化为 0
  Eigen::Vector2d zero(0.0, 0.0);
  std::fill(gradient.begin(), gradient.end(), zero);    // 梯度初始化为 0

  Eigen::Vector2d delta_now, delta_next;
  double norm_now, norm_next, inv_norm_now, inv_norm_next;
  double delta_phi;

  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - 1;
  // int end_idx = q.size();

  for(int i = 0; i < end_idx - 2; i++) {
    delta_now = q[i+1].transpose() - q[i+0].transpose();
    delta_next = q[i+2].transpose() - q[i+1].transpose();
    norm_now = delta_now.norm();
    norm_next = delta_next.norm();

    if (norm_next < 1e-1 || norm_now < 1e-1)
    {
      continue;
    }

    inv_norm_next = 1.0 / norm_next;
    inv_norm_now = 1.0 / norm_now;
    delta_phi = acos(delta_now.dot(delta_next) * inv_norm_now * inv_norm_next);

    double max_kappa_ = 4.0;
    double cost_c = delta_phi * inv_norm_now - max_kappa_;

    if (cost_c > 0 && delta_phi < M_PI_2)
    {
      // ROS_WARN("SHITTING ANGLE IS %f, SHITTING NORM ARE %f, %f", delta_phi, norm_now, norm_next);
      cost += pow(cost_c, 2);

      double rd_cd = 1.0 / sqrt(1 - pow(cos(delta_phi), 2)) * inv_norm_now;

      double inv_norm_12 = inv_norm_next*inv_norm_now;

      Eigen::Vector2d p1 = (delta_now - delta_now.dot(delta_next) * pow(inv_norm_next, 2) * delta_next) * inv_norm_12;
      Eigen::Vector2d p2 = (-delta_next + delta_now.dot(delta_next) * pow(inv_norm_now, 2) * delta_now) * inv_norm_12;

      gradient[i + 0] += 2 * cost_c * (rd_cd * p2 + delta_phi * pow(inv_norm_now, 2) * Eigen::Vector2d(1, 0));
      gradient[i + 1] += 2 * cost_c * (-rd_cd * (p1 + p2) - delta_phi * pow(inv_norm_now, 2) * Eigen::Vector2d(1, 0));
      gradient[i + 2] += 2 * cost_c * rd_cd * p1;
    }
  }
}

void Bspline_Opt::setControlPoints(const Eigen::MatrixXd& points) {
  control_points_ = points;
  dim_            = control_points_.cols();     // xy维度，2
}

void Bspline_Opt::setBsplineInterval(const double& ts) { bspline_interval_ = ts; }

void Bspline_Opt::setTerminateCond(const int& max_num_id, const int& max_time_id) {
  max_num_id_  = max_num_id;
  max_time_id_ = max_time_id;
}

void Bspline_Opt::setCostFunction(const int& cost_code) {
  cost_function_ = cost_code;
  // print optimized cost function
  string cost_str;
  if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";
  if (cost_function_ & DISTANCE) cost_str += " dist  |";
  if (cost_function_ & FEASIBILITY) cost_str += " feasi |";
  if (cost_function_ & ENDPOINT) cost_str += " endpt |";
  if (cost_function_ & GUIDE) cost_str += " guide |";
  if (cost_function_ & WAYPOINTS) cost_str += " waypt |";
  if (cost_function_ & CURVATURE) cost_str += " cutvature |";
  if (cost_function_ & VELOCITYOBSTACLE) cost_str += " VO |";

  ROS_INFO_STREAM("cost func: " << cost_str);
}

void Bspline_Opt::setGuidePath(const vector<Eigen::Vector2d>& guide_pt) { guide_pts_ = guide_pt; }

void Bspline_Opt::setWaypoints(const vector<Eigen::Vector2d>& waypts, const vector<int>& waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

void Bspline_Opt::optimize() {
  /* initialize solver */
  iter_num_        = 0;
  min_cost_        = std::numeric_limits<double>::max();
  const int pt_num = control_points_.rows();
  g_q_.resize(pt_num);
  g_smoothness_.resize(pt_num);
  g_distance_.resize(pt_num);
  g_feasibility_.resize(pt_num);
  g_endpoint_.resize(pt_num);
  g_waypoints_.resize(pt_num);
  g_guide_.resize(pt_num);
  g_culvature_.resize(pt_num);
  g_vo_.resize(pt_num);

  if (cost_function_ & ENDPOINT) {
    variable_num_ = dim_ * (pt_num - order_);
    // end position used for hard constraint
    end_pt_ = (1 / 6.0) *
        (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
         control_points_.row(pt_num - 1));
  } else {
    variable_num_ = max(0, dim_ * (pt_num - 2 * order_)) ;
  }

  /* do optimization using NLopt slover */
  nlopt::opt opt(nlopt::algorithm(algorithm2_), variable_num_);

  opt.set_min_objective(Bspline_Opt::costFunction, this);
  opt.set_maxeval(max_iteration_num_[max_num_id_]);
  opt.set_maxtime(max_iteration_time_[max_time_id_]);
  opt.set_xtol_rel(1e-5);

  vector<double> q(variable_num_);
  for (int i = order_; i < pt_num; ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      q[dim_ * (i - order_) + j] = control_points_(i, j);
    }
  }

  if (dim_ != 1) {
    vector<double> lb(variable_num_), ub(variable_num_);
    const double   bound = 5.0;
    for (int i = 0; i < variable_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  try {
    double          final_cost;
    opt.optimize(q, final_cost);
  } catch (std::exception& e) {
    ROS_WARN("[Optimization]: nlopt exception");
    cout << e.what() << endl;
  }

  for (int i = order_; i < control_points_.rows(); ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];
    }
  }

  if (!(cost_function_ & GUIDE)) ROS_INFO_STREAM("NLOPT iter num: " << iter_num_);
}

Eigen::MatrixXd Bspline_Opt::getControlPoints() { return this->control_points_; }

vector<Eigen::Vector2d> Bspline_Opt::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  vector<Eigen::Vector2d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}



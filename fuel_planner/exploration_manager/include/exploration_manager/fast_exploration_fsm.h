#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::string;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH };

class FastExplorationFSM {
private:
  /* planning utils */
  //链接到expl_manager_中planner_manager_的地址，主要用于存储规划结果
  //局部规划采用FastPlanner算法
  shared_ptr<FastPlannerManager> planner_manager_;
  //对exploration全过程进行管理
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  /* helper functions */
  //**调用自主探索规划器**，重要！！
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);

  /* ROS functions */
  //进行机器人状态管理与切换，频率：100Hz
  void FSMCallback(const ros::TimerEvent& e);
  //进行机器人安全检查，频率：20Hz
  //逻辑：EXEC_TRAJ时进行碰撞检测，如果检测到碰撞则切换至PLAN_TRAJ
  void safetyCallback(const ros::TimerEvent& e);
  //在WAIT_TRIGGER和FINISH进行FIS检查与更新，并将FIS进行可视化显示
  //REMARKS: 1.FINISH时FIS应为空，没必要进行可视化显示；
  //2.WAIT_TRIGGER时计算并显示Frontier
  void frontierCallback(const ros::TimerEvent& e);
  //接收到waypoint后切换状态：由WAIT_TRIGGER到PLAN_TRAJ
  void triggerCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void visualize();
  void clearVisMarker();

public:
  FastExplorationFSM(/* args */) {
  }
  ~FastExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif
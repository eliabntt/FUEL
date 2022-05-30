#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

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

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PLAN_MAN_TRAJ, MAN_GOAL_REACHED, PUB_TRAJ, EXEC_TRAJ, FINISH, PUB_360, PUB_FIRST_360};

class FastExplorationFSM {
private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
	bool has_done_360_ = false;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_, manual_goal_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_, trajectory_pub_, state_pub_, nmpc_pose_pub_, pose_pub_;
	bool is_first_command = true;
	bool is_first_360 = false;
	bool keep_old_goal = false;

  /* helper functions */
  int callExplorationPlanner();
	bool callKinodynamicReplan();
  void transitState(EXPL_STATE new_state, string pos_call);
	void setStartTraj();

  /* ROS functions */
	void setVisInfo(const ros::Time& time_r);
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
	void pubState();
  void visualize();
  void clearVisMarker();
	bool hasMoved();
	double find_tr();
	Eigen::Vector3d old_odom_pos_;
	Eigen::Quaterniond old_odom_orient_;

	int try_count_ = 0;

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
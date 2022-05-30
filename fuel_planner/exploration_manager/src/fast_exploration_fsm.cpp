
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

using Eigen::Vector4d;

namespace fast_planner {
		void FastExplorationFSM::init(ros::NodeHandle &nh) {
			fp_.reset(new FSMParam);
			fd_.reset(new FSMData);

			/*  Fsm param  */
			nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
			nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
			nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
			nh.param("fsm/replan_time", fp_->replan_time_, -1.0);

			/* Initialize main modules */
			expl_manager_.reset(new FastExplorationManager);
			expl_manager_->initialize(nh);
			visualization_.reset(new PlanningVisualization(nh));

			planner_manager_ = expl_manager_->planner_manager_;
			state_ = EXPL_STATE::INIT;
			fd_->have_odom_ = false;
			fd_->state_str_ = {"INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PLAN_MAN_TRAJ", "MAN_GOAL_REACHED", "PUB_TRAJ",
			                   "EXEC_TRAJ", "FINISH", "PUB_360", "PUB_FIRST_360"};
			fd_->static_state_ = true;
			fd_->trigger_ = false;
			fd_->manual_ = false;
			fd_->newest_accelerations_.clear();
			fd_->newest_velocities_.clear();
			planner_manager_->is_doing_360_ = false;
			has_done_360_ = false;

			/* Ros sub, pub and timer */
			exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
			safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
			frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this);
			trigger_sub_ = nh.subscribe("waypoint_generator/waypoints", 1, &FastExplorationFSM::triggerCallback, this);
			odom_sub_ = nh.subscribe("odom_world", 1, &FastExplorationFSM::odometryCallback, this);
			manual_goal_sub_ = nh.subscribe("manual_goal", 1, &FastExplorationFSM::poseCallback, this);

			replan_pub_ = nh.advertise<std_msgs::Empty>("planning/replan", 10);
			new_pub_ = nh.advertise<std_msgs::Empty>("planning/new", 10);
			bspline_pub_ = nh.advertise<bspline::Bspline>("planning/bspline", 10);
			trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1);
			pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
			state_pub_ = nh.advertise<std_msgs::String>("fsm_exploration/state", 1);
			nmpc_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);

		}

		bool FastExplorationFSM::hasMoved() {
			if (fd_->have_odom_) {
				double dx = (fd_->odom_pos_[0] - old_odom_pos_[0]) * (fd_->odom_pos_[0] - old_odom_pos_[0]);
				double dy = (fd_->odom_pos_[1] - old_odom_pos_[1]) * (fd_->odom_pos_[1] - old_odom_pos_[1]);
				double dz = (fd_->odom_pos_[2] - old_odom_pos_[2]) * (fd_->odom_pos_[2] - old_odom_pos_[2]);
				double dist = sqrt(dx + dy + dz);
				if (dist < 0.2) {
					return false;
				} else {
					return true;
				}
			} else {
				return false;
			}
		}

		double FastExplorationFSM::find_tr() {
			LocalTrajData *info = &planner_manager_->local_data_;
			// compute the index such that the distance between info->position_traj_.evaluateDeBoorT(i) and fd_->odom_pos_ is min
			double min_dist = std::numeric_limits<double>::max();
			double min_time = 0;
			// not super beautiful, but it works
			for (double i = 0; i < info->position_traj_.getLength(0.1); i += 0.1) {
				double dx = (info->position_traj_.evaluateDeBoorT(i)[0] - fd_->odom_pos_[0]) *
				            (info->position_traj_.evaluateDeBoorT(i)[0] - fd_->odom_pos_[0]);
				double dy = (info->position_traj_.evaluateDeBoorT(i)[1] - fd_->odom_pos_[1]) *
				            (info->position_traj_.evaluateDeBoorT(i)[1] - fd_->odom_pos_[1]);
				double dz = (info->position_traj_.evaluateDeBoorT(i)[2] - fd_->odom_pos_[2]) *
				            (info->position_traj_.evaluateDeBoorT(i)[2] - fd_->odom_pos_[2]);
				double dist = sqrt(dx + dy + dz);
				if (dist < min_dist) {
					min_dist = dist;
					min_time = i;
				} else {
					return min_time;
				}
			}
			return min_time;
		}

		void FastExplorationFSM::pubState() {
			std_msgs::String msg;
			msg.data = fd_->state_str_[int(state_)];
			state_pub_.publish(msg);
		}

		void FastExplorationFSM::FSMCallback(const ros::TimerEvent &e) {
			ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << fd_->state_str_[int(state_)]);
			pubState();
			switch (state_) {
				case INIT: {
					// Wait for odometry ready
					if (!fd_->have_odom_) {
						ROS_WARN_THROTTLE(1.0, "no odom.");
						return;
					}
					// Go to wait trigger when odom is ok
					transitState(WAIT_TRIGGER, "FSM");
					break;
				}

				case WAIT_TRIGGER: {
					// Do nothing but wait for trigger
					ROS_WARN_THROTTLE(1.0, "wait for trigger.");
					break;
				}

				case FINISH: {
					ROS_INFO_THROTTLE(1.0, "finish exploration.");
					transitState(WAIT_TRIGGER, "FSM");
					break;
				}

				case MAN_GOAL_REACHED: {
					ROS_INFO_THROTTLE(1.0, "goal reached.");
					transitState(WAIT_TRIGGER, "FSM");
					break;
				}

				case PLAN_MAN_TRAJ: {
					geometry_msgs::PoseStamped current_pose;
					current_pose.header.stamp = ros::Time().now();
					current_pose.header.frame_id = "world";
					current_pose.pose.position.x = fd_->odom_pos_(0);
					current_pose.pose.position.y = fd_->odom_pos_(1);
					current_pose.pose.position.z = fd_->odom_pos_(2);
					current_pose.pose.orientation.w = fd_->odom_orient_.w();
					current_pose.pose.orientation.x = fd_->odom_orient_.x();
					current_pose.pose.orientation.y = fd_->odom_orient_.y();
					current_pose.pose.orientation.z = fd_->odom_orient_.z();
					pose_pub_.publish(current_pose);
					ros::Duration(0.2).sleep();


					setStartTraj();

					bool success = callKinodynamicReplan();
					if (success) {
						planner_manager_->is_doing_360_ = false;
						has_done_360_ = false;
						fd_->static_state_ = false;
						transitState(PUB_TRAJ, "FSM");
						try_count_ = 0;
					} else {
						if (try_count_ < 5) {
							if (try_count_ >= 3) {
								planner_manager_->kino_path_finder_->optimistic_ = true;
							}
							transitState(PLAN_MAN_TRAJ, "FSM");
							try_count_++;
						} else {
							has_done_360_ = true;
							fd_->static_state_ = false;
							planner_manager_->is_doing_360_ = true;
							transitState(PUB_360, "FSM");
							try_count_ = 0;
						}
					}
					break;
				}

				case PLAN_TRAJ: {
					geometry_msgs::PoseStamped current_pose;
					current_pose.header.stamp = ros::Time().now();
					current_pose.header.frame_id = "world";
					current_pose.pose.position.x = fd_->odom_pos_(0);
					current_pose.pose.position.y = fd_->odom_pos_(1);
					current_pose.pose.position.z = fd_->odom_pos_(2);
					current_pose.pose.orientation.w = fd_->odom_orient_.w();
					current_pose.pose.orientation.x = fd_->odom_orient_.x();
					current_pose.pose.orientation.y = fd_->odom_orient_.y();
					current_pose.pose.orientation.z = fd_->odom_orient_.z();
					pose_pub_.publish(current_pose);
					ros::Duration(0.2).sleep();
					setStartTraj();

					// Inform traj_server the replanning
					replan_pub_.publish(std_msgs::Empty());
					int res = callExplorationPlanner();
					if (res == SUCCEED) {
						planner_manager_->is_doing_360_ = false;
						transitState(PUB_TRAJ, "FSM");
					} else {
						if (try_count_ < 5) {
							ROS_WARN_STREAM("RETRYING \n");
							if (try_count_ >= 3) {
								planner_manager_->kino_path_finder_->optimistic_ = true;
							}
							transitState(PLAN_TRAJ, "FSM");
							try_count_++;
						} else {
							ROS_WARN_STREAM("TRYING DOING 360");
							ROS_WARN_STREAM("DOING 360");
							has_done_360_ = true;
							fd_->static_state_ = false;
							planner_manager_->is_doing_360_ = true;
							transitState(PUB_360, "FSM");
							try_count_ = 0;
						}
					}
					break;
				}
				case PUB_FIRST_360: {
					transitState(PUB_360, "FSM");
					break;
				}

				case PUB_360: {
					// safely start an in - place rotation
					Eigen::Quaterniond q;
					double start_yaw = std::fmod(fd_->odom_yaw_ + 3.1415926 * 2, 3.1415926 * 2);
					double yaw;

					trajectory_msgs::MultiDOFJointTrajectory msg;
					msg.header.frame_id = "world";
					msg.header.stamp = ros::Time().now();
					geometry_msgs::Transform transform;
					geometry_msgs::Twist velocity;
					geometry_msgs::Twist acceleration;
					int max_i = 4;
					int divider = 360 / max_i; // 24
					for (int i = 0; i < max_i; i++) { // do something less than 360
						trajectory_msgs::MultiDOFJointTrajectoryPoint temp_point;
						transform.translation.x = fd_->odom_pos_.x();
						transform.translation.y = fd_->odom_pos_.y();
						transform.translation.z = fd_->odom_pos_.z();

						yaw = std::fmod(0.2 + start_yaw + i * divider * 3.1415926 / 180, 3.1415926 * 2);
//						yaw = std::fmod(divider * 3.1415926 / 180, 3.1415926 * 2);
						ROS_WARN_STREAM("THE YAW IS " << yaw << " \n");
						q = Eigen::AngleAxisd(yaw, Vector3d::UnitZ());
						transform.rotation.x = q.x();
						transform.rotation.y = q.y();
						transform.rotation.z = q.z();
						transform.rotation.w = q.w();

						velocity.linear.x = 0;
						velocity.linear.y = 0;
						velocity.linear.z = 0;
						velocity.angular.z = 2;
						acceleration.linear.x = 0;
						acceleration.linear.y = 0;
						acceleration.linear.z = 0;

						temp_point.velocities.push_back(velocity);
						temp_point.accelerations.push_back(acceleration);
						temp_point.transforms.push_back(transform);
						temp_point.time_from_start = ros::Duration().fromSec(0.3 * i);
						msg.points.push_back(temp_point);
					}
					planner_manager_->local_data_.duration_ = 1;
					trajectory_pub_.publish(msg);
					fd_->static_state_ = false;
					old_odom_orient_ = fd_->odom_orient_;
					old_odom_pos_ = fd_->odom_pos_;
					transitState(EXEC_TRAJ, "FSM");

					thread vis_thread(&FastExplorationFSM::visualize, this);
					vis_thread.detach();
					planner_manager_->local_data_.start_time_ = ros::Time().now();
					ROS_WARN_STREAM("PUB 360");
					break;
				}

				case PUB_TRAJ: {
					double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
					if (dt > 0) {
						visualization_->drawYawTraj(planner_manager_->local_data_.position_traj_,
						                            planner_manager_->local_data_.yaw_traj_, fd_->newest_traj_.yaw_dt);

						bspline_pub_.publish(fd_->newest_traj_);
						trajectory_msgs::MultiDOFJointTrajectory msg;
						msg.header.frame_id = "world";
						msg.header.stamp = ros::Time().now();
						geometry_msgs::Transform transform;
						geometry_msgs::Twist velocity;
						geometry_msgs::Twist acceleration;
						Eigen::Quaterniond q;
						double step_time = fd_->newest_velocities_.size() * fd_->newest_traj_.yaw_dt * 0.5 / fd_->newest_traj_.pos_pts.size();
						for (int i = 0; i < fd_->newest_traj_.pos_pts.size(); i++) {
							trajectory_msgs::MultiDOFJointTrajectoryPoint temp_point;
							transform.translation.x = fd_->newest_traj_.pos_pts.at(i).x;
							transform.translation.y = fd_->newest_traj_.pos_pts.at(i).y;
							transform.translation.z = fd_->newest_traj_.pos_pts.at(i).z;
							double yaw;
							if (i >= fd_->newest_traj_.yaw_pts.size()) {
								yaw = fd_->newest_traj_.yaw_pts.at(fd_->newest_traj_.yaw_pts.size() - 1);
							} else {
								yaw = fd_->newest_traj_.yaw_pts.at(i);
							}
							q = Eigen::AngleAxisd(yaw, Vector3d::UnitZ());
							transform.rotation.x = q.x();
							transform.rotation.y = q.y();
							transform.rotation.z = q.z();
							transform.rotation.w = q.w();

							if (true) {
								// just override this
								velocity.linear.x = fd_->odom_vel_.x();
								velocity.linear.y = fd_->odom_vel_.y();
								velocity.linear.z = fd_->odom_vel_.z();
								acceleration.linear.x = 0;
								acceleration.linear.y = 0;
								acceleration.linear.z = 0;
							} else {
								// here the = is missing because the first setpoint (i=0) is updated above
								auto cp_vel = planner_manager_->local_data_.velocity_traj_.getControlPoint();
								if (i > fd_->newest_velocities_.size()) {
									velocity.linear.x = cp_vel(cp_vel.rows() - 1, 0);
									velocity.linear.y = cp_vel(cp_vel.rows() - 1, 1);
									velocity.linear.z = cp_vel(cp_vel.rows() - 1, 2);
								} else {
									velocity.linear.x = cp_vel(i - 1, 0);
									velocity.linear.y = cp_vel(i - 1, 1);
									velocity.linear.z = cp_vel(i - 1, 2);
								}
								auto cp_acc = planner_manager_->local_data_.velocity_traj_.getControlPoint();
								if (i > fd_->newest_accelerations_.size()) {
									acceleration.linear.x = cp_acc(cp_acc.rows() - 1, 0);
									acceleration.linear.y = cp_acc(cp_acc.rows() - 1, 1);
									acceleration.linear.z = cp_acc(cp_acc.rows() - 1, 2);
								} else {
									acceleration.linear.x = cp_acc(i - 1, 0);
									acceleration.linear.y = cp_acc(i - 1, 1);
									acceleration.linear.z = cp_acc(i - 1, 2);
								}
							}

							temp_point.velocities.push_back(velocity);
							temp_point.accelerations.push_back(acceleration);
							temp_point.transforms.push_back(transform);
							temp_point.time_from_start = ros::Duration().fromSec(step_time * i);
							msg.points.push_back(temp_point);
						}
						trajectory_pub_.publish(msg);
						fd_->static_state_ = false;
						old_odom_orient_ = fd_->odom_orient_;
						old_odom_pos_ = fd_->odom_pos_;
						transitState(EXEC_TRAJ, "FSM");

						thread vis_thread(&FastExplorationFSM::visualize, this);
						vis_thread.detach();
					}
					break;
				}

				case EXEC_TRAJ: {
					planner_manager_->kino_path_finder_->optimistic_ = false;

					LocalTrajData *info = &planner_manager_->local_data_;
					double t_cur = (ros::Time::now() - info->start_time_).toSec(); // elapsed

					// Replan if traj is almost fully executed
					double time_to_end = info->duration_ - t_cur;
					if (time_to_end < fp_->replan_thresh1_) {
						if (is_first_360) {
							is_first_360 = false;
							transitState(WAIT_TRIGGER, "FSM");
						} else {
							keep_old_goal = false;
							planner_manager_->is_doing_360_ = false;
							has_done_360_ = false;
							if (fd_->manual_) {
								transitState(MAN_GOAL_REACHED, "FSM");
//							fd_->manual_ = false;
							} else
								transitState(PLAN_TRAJ, "FSM");
						}

						ROS_WARN("Replan: traj fully executed=================================");
						break;
					}

					if (!fd_->manual_) {
						// Replan if next frontier to be visited is covered
						if (expl_manager_->frontier_finder_ != nullptr && not is_first_360)
							planner_manager_->is_doing_360_ = false;
						if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
							keep_old_goal = false;
							transitState(PLAN_TRAJ, "FSM");
							ROS_WARN("Replan: cluster covered=====================================");
							return;
						} else {
							keep_old_goal = true;
							transitState(PLAN_TRAJ, "FSM");
							ROS_WARN("Replan: but cluster not covered =====================================");
							return;
						}
					} else {
						if (t_cur > fp_->replan_thresh2_) {
							keep_old_goal = true;
							planner_manager_->is_doing_360_ = false;
							transitState(PLAN_MAN_TRAJ, "FSM");
							ROS_WARN("Replan: safety replan =====================================");
							return;
						}
					}

					// Replan after some time
					if (t_cur > fp_->replan_thresh3_ && !classic_) {
						if (planner_manager_->is_doing_360_) {
							break;
						} else if (fd_->manual_) {
							keep_old_goal = true;
							transitState(PLAN_MAN_TRAJ, "FSM");
						} else {
							keep_old_goal = true;
							transitState(PLAN_TRAJ, "FSM");
						}
						ROS_WARN("Replan: periodic call=======================================");
					}
					break;
				}
			}
		}

		void FastExplorationFSM::setStartTraj() {
			if (!fd_->static_state_ && !hasMoved()) {
				fd_->static_state_ = true;
			}
			if (planner_manager_->is_doing_360_)
				fd_->static_state_ = true;
			if (fd_->static_state_) {
				// Plan from static state (hover)
				fd_->start_pt_ = fd_->odom_pos_;
				fd_->start_vel_ = fd_->odom_vel_;
				fd_->start_acc_.setZero();

				fd_->start_yaw_(0) = fd_->odom_yaw_;
				fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
			} else {
				// Replan from non-static state, starting from 'replan_time' seconds later
				LocalTrajData *info = &planner_manager_->local_data_;
				double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

				fd_->start_pt_ = fd_->odom_pos_;
				fd_->start_vel_ = fd_->odom_vel_;
				try {
					t_r = find_tr();
					fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
					fd_->start_yaw_(0) = fd_->odom_yaw_;
					fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
					fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
				}
				catch (...) {
					fd_->start_acc_.setZero();
					fd_->start_yaw_(0) = fd_->odom_yaw_;
					fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
				}
			}
		}

		int FastExplorationFSM::callExplorationPlanner() {
			ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);
			int res;
			if (!keep_old_goal)
				res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
				                                       fd_->start_yaw_);
			else {
				res = planner_manager_->kinodynamicReplan(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
				                                          fd_->end_pt_, fd_->end_vel_);
				planner_manager_->planYawExplore(fd_->start_yaw_, fd_->end_yaw_, true, expl_manager_->ep_->relax_time_);
			}
			classic_ = false;

			// int res = expl_manager_->classicFrontier(fd_->start_pt_, fd_->start_yaw_[0]);
			// classic_ = true;

			// int res = expl_manager_->rapidFrontier(fd_->start_pt_, fd_->start_vel_, fd_->start_yaw_[0],
			// classic_);
			if (res == SUCCEED) {
				setVisInfo(time_r);
			}
			return res;
		}

		bool FastExplorationFSM::callKinodynamicReplan() {
			auto t1 = ros::Time::now();
			ros::Time time_r = ros::Time::now();
			bool plan_success =
				planner_manager_->kinodynamicReplan(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
				                                    fd_->end_pt_, fd_->end_vel_);
			auto t2 = ros::Time::now();

			if (plan_success) {
				planner_manager_->planYawExplore(fd_->start_yaw_, fd_->end_yaw_, true, expl_manager_->ep_->relax_time_);
				setVisInfo(time_r);
				return true;
			} else {
				cout << "generate new traj fail." << endl;
				return false;
			}
		}

		void FastExplorationFSM::visualize() {
			auto info = &planner_manager_->local_data_;
			auto plan_data = &planner_manager_->plan_data_;
			auto ed_ptr = expl_manager_->ed_;

			// Draw updated box
			// Vector3d bmin, bmax;
			// planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax);
			// visualization_->drawBox((bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0,
			// 4);

			// Draw frontier
			static int last_ftr_num = 0;
			for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
				visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
				                          visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4),
				                          "frontier", i, 4);
			}
			for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
				visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
			}
			last_ftr_num = ed_ptr->frontiers_.size();
			// for (int i = 0; i < ed_ptr->dead_frontiers_.size(); ++i)
			//   visualization_->drawCubes(ed_ptr->dead_frontiers_[i], 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier",
			//                             i, 4);
			// for (int i = ed_ptr->dead_frontiers_.size(); i < 5; ++i)
			//   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);

			// Draw global top viewpoints info
			// visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
			// visualization_->drawLines(ed_ptr->global_tour_, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
			// visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
			// visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1),
			// "point-average", 0, 6);

			// Draw local refined viewpoints info
			// visualization_->drawSpheres(ed_ptr->refined_points_, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
			// visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05,
			//                           Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
			// visualization_->drawLines(ed_ptr->refined_tour_, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
			// visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0, 0, 0,
			// 1),
			//                           "refined_view", 0, 6);
			// visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05, Vector4d(1, 1,
			// 0, 1),
			//                           "refine_pair", 0, 6);
			// for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
			//   visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
			//                               visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
			//                               ed_ptr->frontiers_.size()),
			//                               "n_points", i, 6);
			// for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
			//   visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

			// Draw trajectory
			visualization_->drawBspline(info->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
			                            Vector4d(1, 1, 0, 1));
		}

		void FastExplorationFSM::clearVisMarker() {
			// visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
			// visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
			// visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
			// visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
			// visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
			// visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

			// visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
		}

		void FastExplorationFSM::frontierCallback(const ros::TimerEvent &e) {
			static int delay = 0;
			if (++delay < 5) return;

			if (state_ == WAIT_TRIGGER || state_ == FINISH) {
				auto ft = expl_manager_->frontier_finder_;
				auto ed = expl_manager_->ed_;
				ft->searchFrontiers();
				ft->computeFrontiersToVisit();
				ft->updateFrontierCostMatrix();

				ft->getFrontiers(ed->frontiers_);
				ft->getFrontierBoxes(ed->frontier_boxes_);

				// Draw frontier and bounding box
				for (int i = 0; i < ed->frontiers_.size(); ++i) {
					visualization_->drawCubes(ed->frontiers_[i], 0.1,
					                          visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4),
					                          "frontier", i, 4);
					// visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
					// Vector4d(0.5, 0, 1, 0.3),
					//                         "frontier_boxes", i, 4);
				}
				for (int i = ed->frontiers_.size(); i < 50; ++i) {
					visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
					// visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
					// "frontier_boxes", i, 4);
				}
			}

			// if (!fd_->static_state_)
			// {
			//   static double astar_time = 0.0;
			//   static int astar_num = 0;
			//   auto t1 = ros::Time::now();

			//   planner_manager_->path_finder_->reset();
			//   planner_manager_->path_finder_->setResolution(0.4);
			//   if (planner_manager_->path_finder_->search(fd_->odom_pos_, Vector3d(-5, 0, 1)))
			//   {
			//     auto path = planner_manager_->path_finder_->getPath();
			//     visualization_->drawLines(path, 0.05, Vector4d(1, 0, 0, 1), "astar", 0, 6);
			//     auto visit = planner_manager_->path_finder_->getVisited();
			//     visualization_->drawCubes(visit, 0.3, Vector4d(0, 0, 1, 0.4), "astar-visit", 0, 6);
			//   }
			//   astar_num += 1;
			//   astar_time = (ros::Time::now() - t1).toSec();
			//   ROS_WARN("Average astar time: %lf", astar_time);
			// }
		}

		void FastExplorationFSM::setVisInfo(const ros::Time &time_r) {

			auto info = &planner_manager_->local_data_;
			info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

			bspline::Bspline bspline;
			bspline.order = planner_manager_->pp_.bspline_degree_;
			bspline.start_time = info->start_time_;
			bspline.traj_id = info->traj_id_;
			Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

			fd_->end_pt_(0) = pos_pts(pos_pts.rows() - 1, 0);
			fd_->end_pt_(1) = pos_pts(pos_pts.rows() - 1, 1);
			fd_->end_pt_(2) = pos_pts(pos_pts.rows() - 1, 2);
			fd_->end_vel_.setZero();

			for (int i = 0; i < pos_pts.rows(); ++i) {
				geometry_msgs::Point pt;
				pt.x = pos_pts(i, 0);
				pt.y = pos_pts(i, 1);
				pt.z = pos_pts(i, 2);
				bspline.pos_pts.push_back(pt);
			}
			vector <geometry_msgs::Twist> vel_vec;
			Eigen::MatrixXd vel_pts = info->velocity_traj_.getControlPoint();
			for (int i = 0; i < vel_pts.rows(); ++i) {
				geometry_msgs::Twist pt;
				pt.linear.x = pos_pts(i, 0);
				pt.linear.y = pos_pts(i, 1);
				pt.linear.z = pos_pts(i, 2);
				vel_vec.push_back(pt);
			}
			vector <geometry_msgs::Twist> acc_vec;
			Eigen::MatrixXd acc_pts = info->acceleration_traj_.getControlPoint();
			for (int i = 0; i < vel_pts.rows(); ++i) {
				geometry_msgs::Twist pt;
				pt.linear.x = pos_pts(i, 0);
				pt.linear.y = pos_pts(i, 1);
				pt.linear.z = pos_pts(i, 2);
				acc_vec.push_back(pt);
			}

			Eigen::VectorXd knots = info->position_traj_.getKnot();
			for (int i = 0; i < knots.rows(); ++i) {
				bspline.knots.push_back(knots(i));
			}
			Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
			for (int i = 0; i < yaw_pts.rows(); ++i) {
				double yaw = yaw_pts(i, 0);
				bspline.yaw_pts.push_back(yaw);
			}
			bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
			fd_->newest_traj_ = bspline;
			fd_->newest_velocities_ = vel_vec;
			fd_->newest_accelerations_ = acc_vec;
		}

		void FastExplorationFSM::triggerCallback(const nav_msgs::PathConstPtr &msg) {
			if (state_ != WAIT_TRIGGER) return;
			if (is_first_command) {
				ROS_WARN_STREAM("DOING first 360");
				fd_->static_state_ = false;
				planner_manager_->is_doing_360_ = true;
				has_done_360_ = true;
				transitState(PUB_FIRST_360, "triiger callback");
				is_first_command = false;
				is_first_360 = true;
				return;
			}
			fd_->trigger_ = true;
			fd_->manual_ = false;
			cout << "Triggered!" << endl;
			transitState(PLAN_TRAJ, "triggerCallback");
			planner_manager_->kino_path_finder_->optimistic_ = false;
		}

		void FastExplorationFSM::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
			if (msg->pose.position.z < -0.1) return;
			if (is_first_command) {
				ROS_WARN_STREAM("DOING first 360");
				fd_->static_state_ = false;
				planner_manager_->is_doing_360_ = true;
				has_done_360_ = true;
				transitState(PUB_FIRST_360, "pose callback");

				is_first_command = false;
				is_first_360 = true;
				return;
			}
			// safely start an in - place rotation
			geometry_msgs::PoseStamped tmp;
			tmp.pose.position.x = fd_->odom_pos_.x();
			tmp.pose.position.y = fd_->odom_pos_.y();
			tmp.pose.position.z = fd_->odom_pos_.z();
			Eigen::Quaterniond q;
			double yaw = std::fmod(fd_->odom_yaw_ + 3.1415926 * 2, 3.1415926 * 2); // 0.7 is a manual offset to speed up
			q = Eigen::AngleAxisd(yaw, Vector3d::UnitZ());
			tmp.pose.orientation.x = q.x();
			tmp.pose.orientation.y = q.y();
			tmp.pose.orientation.z = q.z();
			tmp.pose.orientation.w = q.w();
			tmp.header.stamp = ros::Time::now();
			tmp.header.frame_id = "world";
			nmpc_pose_pub_.publish(tmp);
			ros::Duration(0.1).sleep();

			fd_->trigger_ = true;
			planner_manager_->kino_path_finder_->optimistic_ = true;
			if (msg->header.frame_id.compare("world") == 0) {
				fd_->manual_ = true;
			} else if (msg->header.frame_id.compare("fixing_manual") == 0) {
				planner_manager_->kino_path_finder_->optimistic_ = false;
				fd_->manual_ = true;
			} else if (msg->header.frame_id.compare("fixing_automatic") == 0) {
				planner_manager_->kino_path_finder_->optimistic_ = false;
				fd_->manual_ = false;
			} else {
				ROS_WARN_STREAM("Mixing modes because robot is struggling \n");
				planner_manager_->kino_path_finder_->optimistic_ = false;
				fd_->manual_ = false;
			}
			cout << "Manual Goal!" << endl;
			fd_->end_pt_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
			fd_->end_vel_.setZero();
			Eigen::Quaterniond end_orient_;
			end_orient_.x() = msg->pose.orientation.x;
			end_orient_.y() = msg->pose.orientation.y;
			end_orient_.z() = msg->pose.orientation.z;
			end_orient_.w() = msg->pose.orientation.w;
			Eigen::Vector3d rot_x = end_orient_.toRotationMatrix().block<3, 1>(0, 0);
			fd_->end_yaw_ = atan2(rot_x(1), rot_x(0));
			ROS_WARN_STREAM("Planning toward " << fd_->end_pt_ << " " << fd_->end_yaw_);
			transitState(PLAN_MAN_TRAJ, "poseCallback");
		}

		void FastExplorationFSM::safetyCallback(const ros::TimerEvent &e) {
			if (state_ == EXPL_STATE::EXEC_TRAJ) {
				// Check safety and trigger replan if necessary
				double dist;
				bool safe = planner_manager_->checkTrajCollision(dist);
				if (!safe) {
					ROS_WARN("Replan: collision detected==================================");
					if (!fd_->manual_)
						transitState(PLAN_TRAJ, "safetyCallback");
					else
						transitState(PLAN_MAN_TRAJ, "safetyCallback");
				}
			}
		}

		void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
			fd_->odom_pos_(0) = msg->pose.pose.position.x;
			fd_->odom_pos_(1) = msg->pose.pose.position.y;
			fd_->odom_pos_(2) = msg->pose.pose.position.z;

			fd_->odom_vel_(0) = msg->twist.twist.linear.x;
			fd_->odom_vel_(1) = msg->twist.twist.linear.y;
			fd_->odom_vel_(2) = msg->twist.twist.linear.z;

			fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
			fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
			fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
			fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

			Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
			fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

			fd_->have_odom_ = true;
		}

		void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
			int pre_s = int(state_);
			state_ = new_state;
			cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]
			     << endl;
		}
}  // namespace fast_planner

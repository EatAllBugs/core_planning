/*
 * path_generation_node.cpp
 *
 *  Created on: Feb 18, 2020
 *      Author: rcj
 */


#include "conformal_lattice_planner/local_planner/local_planner.h"

namespace planning
{
	LocalPlanner::LocalPlanner(int& argc, char ** &argv)
	:private_nh_("~"),
	 goal_index_(0),
	 closest_point_(std::pair<double,int>(0.0,0)),
	 current_pose_received_(false),
	 current_velocity_received_(false),
	 base_waypoints_received_(false),
	 behaviour_received_(false),
	 first_pose_flag_(false),
	 arrive_flag(false)
	{
		google::InitGoogleLogging("local_planning");
		google::ParseCommandLineFlags(&argc, &argv, true);

//		FLAGS_log_dir = log_folder_;

		prev_best_path_ = std::vector<Eigen::VectorXd>{Eigen::VectorXd(),Eigen::VectorXd(),Eigen::VectorXd(),Eigen::VectorXd()};

		private_nh_.param<double>("lane_width", lane_width_, 2.5);

		apollo::cyber::common::GetProtoFromFile(FLAGS_vehicle_config_path, &params_);
		vehicle_param_ = params_.vehicle_param();

		vehicle_length_ = vehicle_param_.length();
		vehicle_width_ = vehicle_param_.width();
		minimum_turning_radius_ = vehicle_param_.min_turn_radius();

		apollo::cyber::common::GetProtoFromFile(FLAGS_conformal_lattice_config_file, &conformal_lattice_config_);

		update_rate_ = conformal_lattice_config_.update_rate();
		WAIT_TIME_BEFORE_START_ = conformal_lattice_config_.wait_time_before_start();
		path_offset_ = conformal_lattice_config_.path_offset();
		INTERP_DISTANCE_RES = conformal_lattice_config_.interp_distance_res();
		num_paths_ = conformal_lattice_config_.num_paths();
		BP_LOOKAHEAD_BASE_ = conformal_lattice_config_.bp_lookahead_base();
		BP_LOOKAHEAD_TIME_ = conformal_lattice_config_.bp_lookahead_time();
		goal_distace_ = conformal_lattice_config_.goal_distace();
		stop_speed_threhold_ = conformal_lattice_config_.stop_speed_threhold();

		current_pose_sub_ = nh_.subscribe("current_pose", 1, &LocalPlanner::currentPoseCallback, this);
		current_velocity_sub_ = nh_.subscribe("current_velocity", 1, &LocalPlanner::currentVelocityCallback, this);
		base_waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &LocalPlanner::baseWaypointsCallback, this);
		behaviour_sub_ = nh_.subscribe("behaviour", 1, &LocalPlanner::behaviourCallback, this);
		tracked_object_sub_ = nh_.subscribe("/detection/fusion_tools/objects", 1, &LocalPlanner::trackedObjectsCallback, this);

		open_loop_speed_pub_ = nh_.advertise<std_msgs::Float64>("open_loop_speed", 1, true);
		best_trajectory_pub_ = nh_.advertise<my_nav_msgs::Lane>("final_waypoints", 1, true);
		colored_paths_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("local_trajectories_eval_rviz", 1);

		path_optimizer_ = new PathOptimizer(vehicle_param_,conformal_lattice_config_);
		velocity_planner_ = new VelocityPlanner(vehicle_param_,conformal_lattice_config_);
		collision_checker_ = new CollisionChecker(vehicle_param_);



		/*outdatafile_.open("/home/rcj/local_speed.txt");
		if(!outdatafile_)
		{
			std::cout<<"file open error"<<std::endl;

		}*/
	}

	LocalPlanner::~LocalPlanner()
	{
		delete path_optimizer_;
		path_optimizer_ = NULL;

		delete velocity_planner_;
		velocity_planner_ = NULL;

		delete collision_checker_;
		collision_checker_ = NULL;

	}
	void LocalPlanner::trackedObjectsCallback(const my_nav_msgs::DetectedObjectArray& tracked_object)
	{
		tracked_object_ = tracked_object;
	}

	void LocalPlanner::behaviourCallback(const my_nav_msgs::DetectedObject& behaviour)
	{
		lead_vehicle_ = behaviour;
		current_behaviour_ = (StateType)(behaviour.behavior_state);
//		goal_index_ = behaviour.id;

		behaviour_received_ = true;
	}

	void LocalPlanner::baseWaypointsCallback(const my_nav_msgs::Lane& msg)
	{
		if(msg.waypoints.size() == 0)
			return;
		base_waypoints_ = msg;
		base_waypoints_received_ = true;
	}

	void LocalPlanner::currentPoseCallback(const geometry_msgs::PoseStamped& msg)
	{
		ego_state_.current_x = msg.pose.position.x;
		ego_state_.current_y = msg.pose.position.y;
		ego_state_.current_z = msg.pose.position.z;
		ego_state_.current_yaw = tf::getYaw(msg.pose.orientation);

		if(first_pose_flag_ == false)
		{
			Eigen::VectorXd x_points(1);
			x_points<<ego_state_.current_x;
			Eigen::VectorXd y_points(1);
			y_points<<ego_state_.current_y;
			Eigen::VectorXd t_points(1);
			t_points<<ego_state_.current_yaw;
			Eigen::VectorXd kappa_points(1);
			kappa_points<<0.0;

			prev_best_path_.emplace_back(x_points);
			prev_best_path_.emplace_back(y_points);
			prev_best_path_.emplace_back(t_points);
			prev_best_path_.emplace_back(kappa_points);
			first_pose_flag_ = true;
		}
		current_pose_received_ = true;
	}

	void LocalPlanner::currentVelocityCallback(const geometry_msgs::TwistStamped& msg)
	{
		ego_state_.current_speed = msg.twist.linear.x;

		current_velocity_received_ = true;
	}

	void LocalPlanner::GetTickCount(struct timespec& t)
	{
		while(clock_gettime(0, &t) == -1);
	}

	double LocalPlanner::GetTimeDiffNow(const struct timespec& curr_t)
	{
		return (curr_t.tv_sec - start_time_.tv_sec) + ((double)(curr_t.tv_nsec - start_time_.tv_nsec)/ 1000000000.0);
	}

	void LocalPlanner::get_closest_index(my_nav_msgs::Lane& waypoints,EgoState& vehicle_state,std::pair<double,int>& closest_point)
	{
		double closest_len = std::numeric_limits<double>::max();
		int closest_index = 0;

		for(int i = closest_point.second;i< waypoints.waypoints.size() && i > -1;)
		{
			double x1 = waypoints.waypoints[i].pose.pose.position.x;
			double y1 = waypoints.waypoints[i].pose.pose.position.y;
			double d1 = std::sqrt(pow((vehicle_state.current_x-x1),2)+pow((vehicle_state.current_y-y1),2));
			if(d1 <= closest_len)
			{
				closest_len = d1;
				closest_index = i;

			/*	if(current_behaviour_ == state::FOLLOW_LANE)
				{
					if(i == waypoints.waypoints.size()-1)
						break;
					double x2 = waypoints.waypoints[i+1].pose.pose.position.x;
					double y2 = waypoints.waypoints[i+1].pose.pose.position.y;
					double d2 = std::sqrt(pow((vehicle_state.current_x-x2),2)+pow((vehicle_state.current_y-y2),2));
					if(d2 >= d1)
						break;
					++i;
				}
				else
				{
					if(i == 0)
						break;
					double x2 = waypoints.waypoints[i-1].pose.pose.position.x;
					double y2 = waypoints.waypoints[i-1].pose.pose.position.y;
					double d2 = std::sqrt(pow((vehicle_state.current_x-x2),2)+pow((vehicle_state.current_y-y2),2));
					if(d2 >= d1)
						break;
					--i;
				}*/

				if(i == waypoints.waypoints.size()-1)
					break;
				double x2 = waypoints.waypoints[i+1].pose.pose.position.x;
				double y2 = waypoints.waypoints[i+1].pose.pose.position.y;
				double d2 = std::sqrt(pow((vehicle_state.current_x-x2),2)+pow((vehicle_state.current_y-y2),2));
				if(d2 >= d1)
					break;
				++i;
			}
		}
		closest_point.first = closest_len;
		closest_point.second = closest_index;
	}

	int LocalPlanner::get_goal_index(my_nav_msgs::Lane& waypoints,std::pair<double,int>& closet_point)
	{
		double arc_length = closet_point.first;
		int wp_index  = closet_point.second;

		if(arc_length > lookahead_)
			return wp_index;

		/*while(true)
		{
			if(current_behaviour_ == state::FOLLOW_LANE && wp_index < waypoints.waypoints.size() - 1)
			{
				double x1 = waypoints.waypoints[wp_index].pose.pose.position.x;
				double y1 = waypoints.waypoints[wp_index].pose.pose.position.y;
				double x2 = waypoints.waypoints[wp_index+1].pose.pose.position.x;
				double y2 = waypoints.waypoints[wp_index+1].pose.pose.position.y;
				arc_length += std::sqrt(pow((x2-x1),2)+pow((y2-y1),2));
				if(arc_length >= lookahead_)
				{
					wp_index++;
					break;
				}
				else
					wp_index++;
			}
			else if(current_behaviour_ == state::BACK && wp_index > 0)
			{
				double x1 = waypoints.waypoints[wp_index].pose.pose.position.x;
				double y1 = waypoints.waypoints[wp_index].pose.pose.position.y;
				double x2 = waypoints.waypoints[wp_index-1].pose.pose.position.x;
				double y2 = waypoints.waypoints[wp_index-1].pose.pose.position.y;
				arc_length += std::sqrt(pow((x2-x1),2)+pow((y2-y1),2));
				if(arc_length >= lookahead_)
				{
					wp_index--;
					break;
				}
				else
					wp_index--;
			}
			if(wp_index == 0 || wp_index == waypoints.waypoints.size() - 1 )
				break;
		}*/
		while(wp_index < waypoints.waypoints.size() - 1)
		{
			double x1 = waypoints.waypoints[wp_index].pose.pose.position.x;
			double y1 = waypoints.waypoints[wp_index].pose.pose.position.y;
			double x2 = waypoints.waypoints[wp_index+1].pose.pose.position.x;
			double y2 = waypoints.waypoints[wp_index+1].pose.pose.position.y;
			arc_length += std::sqrt(pow((x2-x1),2)+pow((y2-y1),2));
			if(arc_length >= lookahead_)
			{
				wp_index++;
				break;
			}
			else
				wp_index++;
		}
		return wp_index;
	}

	std::vector<std::vector<double>> LocalPlanner::get_goal_state_set()
	{
		double goal_state_local_x = base_waypoints_.waypoints[goal_index_].pose.pose.position.x;
		double goal_state_local_y = base_waypoints_.waypoints[goal_index_].pose.pose.position.y;

		goal_state_local_x-=ego_state_.current_x;
		goal_state_local_y-=ego_state_.current_y;

		double goal_local_x = goal_state_local_x*std::cos(ego_state_.current_yaw)+goal_state_local_y*std::sin(ego_state_.current_yaw);
		double goal_local_y = -goal_state_local_x*std::sin(ego_state_.current_yaw)+goal_state_local_y*std::cos(ego_state_.current_yaw);

		double goal_local_yaw = tf::getYaw(base_waypoints_.waypoints[goal_index_].pose.pose.orientation)-ego_state_.current_yaw;
		if(goal_local_yaw > M_PI)
			goal_local_yaw-= 2*M_PI;
		else if(goal_local_yaw < -M_PI)
			goal_local_yaw+=2*M_PI;

		double goal_v = base_waypoints_.waypoints[goal_index_].twist.twist.linear.x;

		std::vector<std::vector<double>> goal_state_set;
		for(int i = 0;i<num_paths_;++i)
		{
			double offset = (i-num_paths_/2)*path_offset_;

			double x_offset = offset*std::cos(goal_local_yaw + M_PI_2);
			double y_offset = offset*std::sin(goal_local_yaw + M_PI_2);

			goal_state_set.emplace_back(std::vector<double>{goal_local_x+x_offset,goal_local_y+y_offset,goal_local_yaw,goal_v});
		}
		return goal_state_set;
	}

	std::vector<std::vector<Eigen::VectorXd>> LocalPlanner::plan_paths(std::vector<std::vector<double>>& goal_state_set)
	{
		std::vector<std::vector<Eigen::VectorXd>> paths;
		if(planner_helper::distance(goal_state_set[num_paths_/2][0],goal_state_set[num_paths_/2][1]) < goal_distace_)
		{
			arrive_flag  = true;
		}
		else if(planner_helper::distance(goal_state_set[num_paths_/2][0],goal_state_set[num_paths_/2][1]) > 0.5)
		{
			arrive_flag  = false;
		}

		if(arrive_flag)
		{
			std::cout<<"get goal"<<std::endl;
			closest_point_ = std::make_pair(0.0,0);
			return paths;
		}

		int i = 0;
		for(auto goal_state : goal_state_set)
		{
			std::vector<Eigen::VectorXd> path = path_optimizer_->optimize_spiral(goal_state,current_behaviour_);
			if(path.size() == 0)
				continue;
			int num_points = path[0].size();
			Eigen::Vector3d diff;
			diff <<path[0](num_points-1)-goal_state[0],path[1](num_points-1)-goal_state[1],
				   path[2](num_points-1)-goal_state[2];
			if(diff.norm()>0.1)
			{
				LOG(INFO) <<"path id:"<<i<<" discard";
			}
			else
				paths.emplace_back(path);
			i++;
		}
		return paths;

	}

	void LocalPlanner::transform_paths(std::vector<std::vector<Eigen::VectorXd>>& paths,EgoState& ego_state)
	{
		for(auto& path : paths)
		{
			for(int i = 0;i<path[0].size();++i)
			{
				double tmp = ego_state.current_x + path[0](i)*std::cos(ego_state.current_yaw) - path[1](i)*std::sin(ego_state.current_yaw);
				path[1](i) = ego_state.current_y + path[0](i)*std::sin(ego_state.current_yaw) + path[1](i)*std::cos(ego_state.current_yaw);
				path[2](i) = path[2](i)+ego_state.current_yaw;
				path[0](i) = tmp;
			}
		}
	}

	int LocalPlanner::select_best_path_index(std::vector<std::vector<Eigen::VectorXd>>& paths,std::vector<bool>& collision_check_array)
	{
		double goal_state_x = base_waypoints_.waypoints[goal_index_].pose.pose.position.x;
		double goal_state_y = base_waypoints_.waypoints[goal_index_].pose.pose.position.y;
		double goal_state_yaw = tf::getYaw(base_waypoints_.waypoints[goal_index_].pose.pose.orientation);

		int best_index = paths.size();
		double best_score = std::numeric_limits<double>::max();
		for(int i = 0;i<paths.size();++i)
		{
			double score;
			if(collision_check_array[i])
			{
				int num_points = paths[i][0].size();
				Eigen::VectorXd diff(2);
				diff<<paths[i][0](num_points-1)-goal_state_x,paths[i][1](num_points-1)-goal_state_y;
				/*if(i < num_paths_/2 && diff.norm() > (lane_width_/2.0 - vehicle_width_/2))
					score = std::numeric_limits<double>::max();
				else*/
					score = diff.norm();
				double dx = paths[i][0](num_points-1)-goal_state_x;
				double dy = paths[i][1](num_points-1)-goal_state_y;

				double angle = std::atan2(dy,dx);
				double sweep_angle = angle -goal_state_yaw;
				if(/*sweep_angle < 0 && */score > 0.2)
					score = std::numeric_limits<double>::max();
			}
			else
				score = std::numeric_limits<double>::max();
			if(score < best_score)
			{
				best_score = score;
				best_index = i;
			}

		}
		return best_index;
	}

	void LocalPlanner::ColoredPathMarkers(std::vector<std::vector<Eigen::VectorXd>>& paths,std::vector<bool>& collision_check_array,int& best_index)
	{
		visualization_msgs::MarkerArray paths_marker;

		visualization_msgs::Marker path_marker;

		path_marker.header.frame_id = "map";
		path_marker.header.stamp = ros::Time();
		path_marker.ns = "local_paths_colored";
		path_marker.type = visualization_msgs::Marker::LINE_STRIP;
		path_marker.action = visualization_msgs::Marker::ADD;
		path_marker.scale.x = 0.1;
		path_marker.scale.y = 0.1;

		path_marker.color.a = 0.9;
		path_marker.frame_locked = false;
		for(unsigned int i = 0; i < paths.size(); ++i)
		{
			path_marker.points.clear();
			path_marker.id = i;

			for(unsigned int j = 0; j < paths[i][0].size(); ++j)
			{
				geometry_msgs::Point point;

				point.x = paths[i][0](j);
				point.y = paths[i][1](j);
				point.z = 0;

				path_marker.points.push_back(point);
			}
			if(i == best_index)
			{
				path_marker.color.r = 0.0;
				path_marker.color.g = 0.0;
				path_marker.color.b = 1.0;

				paths_marker.markers.push_back(path_marker);
				continue;
			}
			if(collision_check_array[i] == true)
			{
				path_marker.color.r = 0.0;
				path_marker.color.g = 1.0;
				path_marker.color.b = 0.0;
			}
			else
			{
				path_marker.color.r = 1.0;
				path_marker.color.g = 0.0;
				path_marker.color.b = 0.0;
			}

			paths_marker.markers.push_back(path_marker);
		}
		colored_paths_pub_.publish(paths_marker);
	}

	void LocalPlanner::BestTrajectoryPub(std::vector<Eigen::VectorXd>& best_path)
	{
		// Linear interpolation computation on the waypoints
		//is also used to ensure a fine resolution between points.
		my_nav_msgs::Lane local_trajectory;
		local_trajectory.header.frame_id ="map";
		std::vector<double> wp_distance;
		for(int i = 1;i < best_path[0].size();++i)
		{
			wp_distance.push_back(planner_helper::distance(best_path[0](i)-best_path[0](i-1),
									best_path[1](i)-best_path[1](i-1)));
		}
		wp_distance.push_back(0.0);

		my_nav_msgs::Waypoint point;
		for(int i = 0;i < static_cast<int>(best_path[0].size()) - 1;++i)
		{
			point.pose.pose.position.x = best_path[0](i);
			point.pose.pose.position.y = best_path[1](i);
			point.pose.pose.orientation = tf::createQuaternionMsgFromYaw(best_path[2](i));

			point.twist.twist.linear.x = best_path[3](i);
			point.twist.twist.linear.z = best_path[4](i);
			local_trajectory.waypoints.push_back(point);

			int num_pts_to_interp = (int)(std::floor(wp_distance[i]/INTERP_DISTANCE_RES)-1);
			Eigen::VectorXd wp_vector(5);
			wp_vector<<best_path[0](i+1)-best_path[0](i),best_path[1](i+1)-best_path[1](i),
					   best_path[2](i+1)-best_path[2](i),best_path[3](i+1)-best_path[3](i),
					   best_path[4](i+1)-best_path[4](i);

			Eigen::VectorXd wp_uvector = wp_vector/wp_vector.norm();

			for(int j = 0;j<num_pts_to_interp;++j)
			{
				Eigen::VectorXd next_wp_vector = INTERP_DISTANCE_RES *(j+1)*wp_uvector;
				Eigen::VectorXd waypoints_i(5);
				waypoints_i<<best_path[0](i),best_path[1](i),best_path[2](i),best_path[3](i),best_path[4](i);

				Eigen::VectorXd interp_point = waypoints_i + next_wp_vector;

				point.pose.pose.position.x = interp_point(0);
				point.pose.pose.position.y = interp_point(1);
				point.pose.pose.orientation = tf::createQuaternionMsgFromYaw(interp_point(2));

				point.twist.twist.linear.x = interp_point(3);
				point.twist.twist.linear.z = interp_point(4);
				local_trajectory.waypoints.push_back(point);
			}
		}
		if(best_path[0].size() != 0)
		{
			point.pose.pose.position.x = best_path[0](best_path[0].size()-1);
			point.pose.pose.position.y = best_path[1](best_path[0].size()-1);
			point.pose.pose.orientation = tf::createQuaternionMsgFromYaw(best_path[2](best_path[0].size()-1));

			point.twist.twist.linear.x = best_path[3](best_path[0].size()-1);
			point.twist.twist.linear.z = best_path[4](best_path[0].size()-1);
			local_trajectory.waypoints.push_back(point);
			LOG(INFO) <<"last point velocity:" <<best_path[3](best_path[0].size()-1);
		}

		best_trajectory_pub_.publish(local_trajectory);

	}
	void LocalPlanner::SpeedPlanningDebug(const Eigen::VectorXd& speeds)
	{
		static int i = 0;
		if(speeds.size() != 0)
		{
			i++;
			outdatafile_<<i<<"  ";
		}
		for(int i = 0;i< speeds.size();++i)
		{
			outdatafile_<<speeds(i)<<" ";
		}
		outdatafile_<<std::endl;

	}

	void LocalPlanner::run()
	{
		static bool stop = false;
		ros::Rate rate(update_rate_);
		GetTickCount(start_time_);
		GetTickCount(current_time_);
		ego_state_.current_timestamp = GetTimeDiffNow(current_time_);

		while (ros::ok())
		{
			ros::spinOnce();
			if (!current_pose_received_ || !current_velocity_received_ || !base_waypoints_received_ || !behaviour_received_)
			{
				LOG(INFO) <<"current message not receiver";
				rate.sleep();
				continue;
			}

//			current_behaviour_ = state::FOLLOW_LANE;
			/*if(flag%2 != 0)
			{
				sleep(1);
				current_behaviour_= state::BACK;

			}*/

			LOG(INFO) <<"current behaviour:(0:INIT): "<<current_behaviour_;

			ego_state_.prev_timestamp = ego_state_.current_timestamp;
			GetTickCount(current_time_);
			ego_state_.current_timestamp = GetTimeDiffNow(current_time_);

			if(ego_state_.current_timestamp <= WAIT_TIME_BEFORE_START_)
				continue;
			else
				ego_state_.current_timestamp = ego_state_.current_timestamp-WAIT_TIME_BEFORE_START_;

			LOG(INFO) <<"------------path generation start------------";

			ego_state_.open_loop_speed = velocity_planner_->get_open_loop_speed(ego_state_.current_timestamp-ego_state_.prev_timestamp);

			LOG(INFO) <<"time interal: "<<ego_state_.current_timestamp-ego_state_.prev_timestamp;

			static unsigned int count = 0;
			static double k = 1.0;

			/*if(prev_best_path_[0].size() != 0)
			{
				unsigned int size = prev_best_path_[0].size();
				double start_yaw = prev_best_path_[2](0);
				double end_yaw = prev_best_path_[2](size-1);
				double err_yaw = end_yaw - start_yaw;
				planner_helper::normalize_angle(err_yaw);
//				std::cout<<"err_yaw: "<<err_yaw<<std::endl;
				if(std::abs(err_yaw) > 0.15)
				{
//					double yaw0 = base_waypoints_.waypoints[goal_index_-1].pose.pose.orientation
					++count;
				    k = -5.0/(lookahead_/std::abs(err_yaw));
//					k = 0.2;
				}
				else
					k = 1.0;
			}

			if(count*(1.0/update_rate_) > lookahead_/ ego_state_.open_loop_speed)
			{
				count = 0;
				k = 1.0;
				std::cout<<"turn end!!"<<std::endl;
			}
			std::cout<<"count: "<<count<<std::endl;*/
			if(!stop)
			{
				if(current_behaviour_ == state::FOLLOW_LANE)
					lookahead_ = BP_LOOKAHEAD_BASE_ + k*BP_LOOKAHEAD_TIME_ * ego_state_.open_loop_speed;
				else if(current_behaviour_ == state::BACK)
					lookahead_ = BP_LOOKAHEAD_BASE_/2.0 + k*BP_LOOKAHEAD_TIME_ * ego_state_.open_loop_speed;
			}

//			std::cout<<"BP_LOOKAHEAD_BASE_: "<<BP_LOOKAHEAD_BASE_<<std::endl;

			get_closest_index(base_waypoints_,ego_state_,closest_point_);
			goal_index_ = get_goal_index(base_waypoints_,closest_point_);

			std::vector<std::vector<double>> goal_state_set = get_goal_state_set();
			LOG(INFO) <<"nums of goal state set: "<<goal_state_set.size();

			auto time_start = std::chrono::system_clock::now();
			std::vector<std::vector<Eigen::VectorXd>> paths = plan_paths(goal_state_set);
			auto time_end = std::chrono::system_clock::now();
			auto usec = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count();

			LOG(INFO) <<"paths time(msec): "<<usec/1000.0;
			LOG(INFO) <<"paths num: "<<paths.size();

			std::vector<std::vector<Eigen::VectorXd>> paths_local = paths;
			transform_paths(paths,ego_state_);

			std::vector<bool> collision_check_array = collision_checker_->collision_check(paths_local,tracked_object_);

			int best_index = select_best_path_index(paths,collision_check_array);
			std::vector<Eigen::VectorXd> best_path(4);//x,y.yaw.kappa.

			if(best_index == paths.size())
			{
				LOG(INFO) <<"no path was feasible";
//				best_path = prev_best_path_;
				stop = true;
			}
			else
			{
				stop = false;
				best_path = paths[best_index];
				prev_best_path_ = best_path;
			}

			ColoredPathMarkers(paths,collision_check_array,best_index);

			LOG(INFO) <<"         velocity profile start          ";

			double desired_speed = base_waypoints_.waypoints[goal_index_].twist.twist.linear.x;
			velocity_planner_->compute_velocity_profile(best_path,desired_speed,ego_state_,current_behaviour_,lead_vehicle_);

//			SpeedPlanningDebug(best_path[3]);

			BestTrajectoryPub(best_path);

			auto time_circle_end = std::chrono::system_clock::now();
			usec = std::chrono::duration_cast<std::chrono::microseconds>(time_circle_end - time_start).count();
			LOG(INFO) <<"total plan time(msec): "<<usec/1000.0;

			behaviour_received_ = false;
			current_pose_received_ = false;
			current_velocity_received_ = false;
			base_waypoints_received_ = false;
			rate.sleep();
		}

	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "conformal_lattice");

  planning::LocalPlanner local_planner(argc,argv);
  local_planner.run();

  return 0;
}


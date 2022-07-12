/*
 * local_planner.h
 *
 *  Created on: Feb 18, 2020
 *      Author: rcj
 */

#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "my_nav_msgs/LaneArray.h"
#include "my_nav_msgs/DetectedObjectArray.h"
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <utility>
#include <fstream>
#include <string>
#include <ctime>
#include <set>
#include <cmath>
#include <limits>
#include <glog/logging.h>
#include <chrono>
#include <unistd.h>

#include "planner_helper/planner_helper.h"

#include "conformal_lattice_config.pb.h"
#include "config_gflags/local_gflags.h"
#include "config_gflags.h"

#include "file.h"

#include "conformal_lattice_planner/path_planning/path_generation.h"
#include "conformal_lattice_planner/velocity_planning/velocity_profile.h"

namespace planning
{
	class LocalPlanner
	{
	public:
		LocalPlanner(int& argc, char ** &argv);
		~LocalPlanner();

		void run();
	private:
		ros::NodeHandle nh_, private_nh_;

		ros::Subscriber current_pose_sub_;
	    ros::Subscriber current_velocity_sub_;
	    ros::Subscriber base_waypoints_sub_;
	    ros::Subscriber behaviour_sub_;
	    ros::Subscriber tracked_object_sub_;

	    ros::Publisher open_loop_speed_pub_;
	    ros::Publisher best_trajectory_pub_;
	    ros::Publisher colored_paths_pub_;

		apollo::common::VehicleConfig params_;
	    apollo::common::VehicleParam vehicle_param_;
	    ConformalLatticeConfig conformal_lattice_config_;

	    bool first_pose_flag_;
	    bool current_pose_received_;
	    bool current_velocity_received_;
	    bool base_waypoints_received_;
	    bool behaviour_received_;

	    int num_paths_;
	    int update_rate_;
	    int goal_index_;
	    int num_points_;
	    bool arrive_flag;
	    EgoState ego_state_;
	    StateType current_behaviour_;

	    double WAIT_TIME_BEFORE_START_;
	    double path_offset_;
	    double minimum_turning_radius_;
	    double INTERP_DISTANCE_RES;
	    double lane_width_;
	    double vehicle_length_;
	    double vehicle_width_;
	    double BP_LOOKAHEAD_BASE_;
	    double BP_LOOKAHEAD_TIME_;
	    double lookahead_;
	    double goal_distace_;
	    double stop_speed_threhold_;

	    my_nav_msgs::DetectedObjectArray tracked_object_;
	    my_nav_msgs::DetectedObject lead_vehicle_;

	    std::vector<Eigen::VectorXd> prev_best_path_;

	    std::ofstream outdatafile_;

	    std::pair<double,int>  closest_point_;

	    struct timespec start_time_;
	    struct timespec current_time_;
	    my_nav_msgs::Lane base_waypoints_;

	    PathOptimizer* path_optimizer_;
	    VelocityPlanner* velocity_planner_;
	    CollisionChecker* collision_checker_;

	    void behaviourCallback(const my_nav_msgs::DetectedObject& behaviour);
	    void baseWaypointsCallback(const my_nav_msgs::Lane& msg);
	    void currentPoseCallback(const geometry_msgs::PoseStamped& msg);
	    void currentVelocityCallback(const geometry_msgs::TwistStamped& msg);
	    void trackedObjectsCallback(const my_nav_msgs::DetectedObjectArray& tracked_object);

	    void GetTickCount(struct timespec& t);
	    double GetTimeDiffNow(const struct timespec& curr_t);
	    void get_closest_index(my_nav_msgs::Lane& waypoints,EgoState& vehicle_state,std::pair<double,int>& closest_point);
	    int get_goal_index(my_nav_msgs::Lane& waypoints,std::pair<double,int>& closet_point);
	    std::vector<std::vector<double>> get_goal_state_set();
	    std::vector<std::vector<Eigen::VectorXd>> plan_paths(std::vector<std::vector<double>>& goal_state_set);
	    void transform_paths(std::vector<std::vector<Eigen::VectorXd>>& paths,EgoState& ego_state);
		int select_best_path_index(std::vector<std::vector<Eigen::VectorXd>>& paths,std::vector<bool>& collision_check_array);
		void ColoredPathMarkers(std::vector<std::vector<Eigen::VectorXd>>& paths,std::vector<bool>& collision_check_array,int& best_index);
		void BestTrajectoryPub(std::vector<Eigen::VectorXd>& best_path);

		void SpeedPlanningDebug(const Eigen::VectorXd& speeds);
	};


}

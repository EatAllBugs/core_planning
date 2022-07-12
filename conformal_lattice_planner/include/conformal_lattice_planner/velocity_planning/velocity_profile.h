/*
 * velocity_profile.h
 *
 *  Created on: Feb 18, 2020
 *      Author: rcj
 */

#pragma once

#include <my_nav_msgs/Lane.h>
#include <my_nav_msgs/Waypoint.h>
#include "my_nav_msgs/DetectedObjectArray.h"

#include <glog/logging.h>

#include <cmath>

#include <Eigen/Dense>

#include "planner_helper/planner_helper.h"

#include "conformal_lattice_config.pb.h"
#include "vehicle_config.pb.h"

namespace planning
{
	class VelocityPlanner
	{
	public:
		VelocityPlanner(apollo::common::VehicleParam& vehicle_param,ConformalLatticeConfig& conformal_lattice_config);
		~VelocityPlanner();
		double get_open_loop_speed(double timestep);
		void compute_velocity_profile(std::vector<Eigen::VectorXd>& best_path,double& desired_speed,EgoState& ego_state,StateType& current_behaviour,my_nav_msgs::DetectedObject& lead_vehicle);

	private:
		double time_gap_;
		double a_max_;
		double a_lat_max_;
		double slow_speed_;
		double stop_line_buffer_;
		double vehicle_length_;
		double obstacle_vel_;
		double obstacle_buffer_;
		my_nav_msgs::Lane prev_trajectory_;

	    double minimum_turning_radius_;

		apollo::common::VehicleParam vehicle_param_;
		ConformalLatticeConfig conformal_lattice_config_;

		double calc_distance(double& v_i,double& v_f,double a);
		double calc_acceleration(double& v_0,double& v_t,double& s);
		double calc_final_speed(double& v_i,double a,double& d);
		void init_profile(std::vector<Eigen::VectorXd>& best_path);
		void decelerate_profile(std::vector<Eigen::VectorXd>& best_path,double& start_speed);
		void follow_profile(std::vector<Eigen::VectorXd>& best_path,double& start_speed,double desired_speed,my_nav_msgs::DetectedObject& lead_vehicle);
		void nominal_profile(std::vector<Eigen::VectorXd>& best_path,double& start_speed,double& desired_speed);
		void emergency_stop_profile(std::vector<Eigen::VectorXd>& best_path,double& start_speed);
		void stop_wait_profile(std::vector<Eigen::VectorXd>& best_path,double& start_speed);
	};
}

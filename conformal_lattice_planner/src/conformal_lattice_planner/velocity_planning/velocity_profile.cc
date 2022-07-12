/*
 * velocity_profile.cpp
 *
 *  Created on: Feb 18, 2020
 *      Author: rcj
 */

#include "conformal_lattice_planner/velocity_planning/velocity_profile.h"

namespace planning
{
	VelocityPlanner::VelocityPlanner(apollo::common::VehicleParam& vehicle_param,ConformalLatticeConfig& conformal_lattice_config)
	:obstacle_vel_(1.5)
	{
		/*my_nav_msgs::Waypoint point;
		point.pose.pose.position.x = 0;
		point.pose.pose.position.y = 0;
		point.twist.twist.linear.x = 0;
		prev_trajectory_.waypoints.push_back(point);*/

		vehicle_param_ = vehicle_param;
		conformal_lattice_config_ = conformal_lattice_config;

		minimum_turning_radius_ = vehicle_param_.min_turn_radius();
		a_max_ = vehicle_param_.max_acceleration();
		vehicle_length_ = vehicle_param_.length();

		time_gap_ = conformal_lattice_config_.velocity_planning_config().time_gap();
		a_lat_max_ = conformal_lattice_config_.velocity_planning_config().a_lat_max();
		slow_speed_ = conformal_lattice_config_.velocity_planning_config().slow_speed();
		stop_line_buffer_ = conformal_lattice_config_.velocity_planning_config().stop_line_buffer();
		obstacle_buffer_ = conformal_lattice_config_.velocity_planning_config().obstacle_buffer();

	}
	VelocityPlanner::~VelocityPlanner()
	{

	}

	double VelocityPlanner::calc_distance(double& v_i,double& v_f,double a)
	{
		return (v_f*v_f - v_i*v_i) / (2 * a);
	}

	double VelocityPlanner::calc_acceleration(double& v_0,double& v_t,double& s)
	{
		return (v_t*v_t - v_0*v_0)/(2*s);
	}
	double VelocityPlanner::calc_final_speed(double& v_i,double a,double& d)
	{
		return std::sqrt(v_i*v_i + 2*a*d);
	}
	double VelocityPlanner::get_open_loop_speed(double timestep)
	{
		if(prev_trajectory_.waypoints.size() == 0)
			return 0.0;
		if(timestep < 1e-4)
			return prev_trajectory_.waypoints[0].twist.twist.linear.x;
		for(int i = 0;i<prev_trajectory_.waypoints.size()-1;++i)
		{
			double diff_x = prev_trajectory_.waypoints[i+1].pose.pose.position.x-
					prev_trajectory_.waypoints[i].pose.pose.position.x;
			double diff_y = prev_trajectory_.waypoints[i+1].pose.pose.position.y-
					prev_trajectory_.waypoints[i].pose.pose.position.y;
			double distance_step = std::sqrt(std::pow(diff_x,2)+std::pow(diff_y,2));

			double velocity = prev_trajectory_.waypoints[i].twist.twist.linear.x;
			double time_delta = distance_step/velocity;

			if(time_delta > timestep)
			{
				double v1 = prev_trajectory_.waypoints[i].twist.twist.linear.x;
				double v2 = prev_trajectory_.waypoints[i+1].twist.twist.linear.x;
				double v_delta = v2 - v1;
				double interpolation_ratio = timestep / time_delta;
				return v1 + interpolation_ratio * v_delta;

			}
			else
				timestep -= time_delta;

		}
		return (*(prev_trajectory_.waypoints.end()-1)).twist.twist.linear.x;

	}
	void VelocityPlanner::init_profile(std::vector<Eigen::VectorXd>& best_path)
	{
		Eigen::VectorXd v_points(best_path[0].size());

		for(int i = 0;i < best_path[0].size();++i)
			v_points(i) = 0.0;
		best_path.insert(best_path.end()-1,v_points);
		return;
	}
	void VelocityPlanner::decelerate_profile(std::vector<Eigen::VectorXd>& best_path,double& start_speed)
	{
		double desired_speed = 0.0;
		double decel_distance = calc_distance(start_speed, slow_speed_, -a_max_);
		double brake_distance = calc_distance(slow_speed_, desired_speed, -a_max_);

		double path_length = 0.0;
		for(int i = 0;i < best_path[0].size()-1;++i)
		{
			path_length += planner_helper::distance(best_path[0](i+1)-best_path[0](i),
													best_path[1](i+1)-best_path[1](i));
		}

		int stop_index = best_path[0].size() - 1;
		double temp_dist = 0.0;
		while(stop_index > 0 && temp_dist < obstacle_buffer_)
		{
			temp_dist += planner_helper::distance(best_path[0](stop_index)-best_path[0](stop_index-1),
												  best_path[1](stop_index)-best_path[1](stop_index-1));
			stop_index--;
		}

		Eigen::VectorXd v_points(best_path[0].size());
		if(brake_distance + decel_distance + obstacle_buffer_ > path_length)
		{
			double vf = 0.0;
			for(int i = best_path[0].size() - 1;i >= stop_index; --i)
			{
				v_points(i) = 0.0;
			}
			for(int i = stop_index - 1;i >= 0;--i)
			{
				double dist = planner_helper::distance(best_path[0](i+1)-best_path[0](i),
													   best_path[1](i+1)-best_path[1](i));
				double vi = calc_final_speed(vf, a_max_, dist);
				if(vi > start_speed)
					vi = start_speed;
				v_points(i) = vi;
				vf = vi;
			}
		}
		else
		{
			int brake_index = stop_index;
			temp_dist = 0.0;
			while(brake_index > 0 && temp_dist < brake_distance)
			{
				temp_dist += planner_helper::distance(best_path[0](brake_index)-best_path[0](brake_index-1),
													  best_path[1](brake_index)-best_path[1](brake_index-1));
				brake_index--;
			}

			int decel_index = 0;
			temp_dist = 0.0;
			while(decel_index < brake_index && temp_dist < decel_distance)
			{
				temp_dist += planner_helper::distance(best_path[0](decel_index+1)-best_path[0](decel_index),
													  best_path[1](decel_index+1)-best_path[1](decel_index));
				decel_index++;
			}

			double vi = start_speed;
			for(int i = 0;i < decel_index;++i)
			{
				double dist = planner_helper::distance(best_path[0](i+1)-best_path[0](i),
													   best_path[1](i+1)-best_path[1](i));
				double vf = calc_final_speed(vi, -a_max_, dist);
				if(vf < slow_speed_)
				{
					vf = slow_speed_;
				}
				v_points(i) = vi;
				vi = vf;
			}
			for(int i = decel_index;i < brake_index;++i)
			{
				v_points(i) = vi;
			}
			for(int i = brake_index; i < stop_index;++i)
			{
				double dist = planner_helper::distance(best_path[0](i+1)-best_path[0](i),
													   best_path[1](i+1)-best_path[1](i));
				double vf = calc_final_speed(vi, -a_max_, dist);
				v_points(i) = vi;
				vi = vf;
			}
			for(int i = stop_index;i < best_path[0].size();++i)
			{
				v_points(i) = 0.0;
			}

		}
		best_path.insert(best_path.end()-1,v_points);

		return;
	}
	void VelocityPlanner::follow_profile(std::vector<Eigen::VectorXd>& best_path,double& start_speed,double desired_speed,my_nav_msgs::DetectedObject& lead_vehicle)
	{
		Eigen::VectorXd v_points(best_path[0].size());
		if(best_path[0].size() == 0)
		{
			best_path.insert(best_path.end()-1,v_points);
			return;
		}
//		std::cout<<"path size: "<<best_path[0].size()<<std::endl;
		double lead_vehicle_x = lead_vehicle.pose.position.x;
		double lead_vehicle_y = lead_vehicle.pose.position.y;
		double lead_vehicle_v = lead_vehicle.velocity.linear.x;
//		std::cout<<"lead_vehicle_v: "<<lead_vehicle_v<<std::endl;

		desired_speed = std::min(lead_vehicle_v, desired_speed);
		double disToLead = planner_helper::distance(0.0-lead_vehicle_x,0.0-lead_vehicle_y);

		int nearest_index = best_path[0].size()-2;
		double nearest_dist = 0.0;
		for(int i = 0;i < best_path[0].size()-1;++i)
		{
			nearest_dist += planner_helper::distance(best_path[0](i+1)-best_path[0](i),
													 best_path[1](i+1)-best_path[1](i));
			if(nearest_dist > disToLead)
			{
				nearest_index = i;
			}
		}

		int ramp_end_index = nearest_index;//3;
		double vi = start_speed;
/*		if(std::abs(start_speed-lead_vehicle_v) < 0.1)
		{
			for(int i = 0;i<ramp_end_index + 1;++i)
			{
				double dist = planner_helper::distance(best_path[0](i+1)-best_path[0](i),
													   best_path[1](i+1)-best_path[1](i));
				double vf;
				if(desired_speed < start_speed)
				{
					vf = calc_final_speed(vi, -a_max_, dist);
					if(vf < desired_speed)
						vf = desired_speed;
				}
				else
				{
					vf = calc_final_speed(vi, a_max_, dist);
					if(vf > desired_speed)
						vf = desired_speed;
				}
				v_points(i) = vi;
				vi = vf;
			}
			for(int i = ramp_end_index+1;i< best_path[0].size();++i)
			{
				v_points(i) = desired_speed;
			}
		}
		else if(lead_vehicle_v > start_speed)
		{
			for(int i = 0;i<ramp_end_index + 1;++i)
			{
				double dist = planner_helper::distance(best_path[0](i+1)-best_path[0](i),
													   best_path[1](i+1)-best_path[1](i));
				double vf = calc_final_speed(vi, a_max_, dist);
				if(vf > desired_speed)
					vf = desired_speed;

				v_points(i) = vi;
				vi = vf;
			}
			for(int i = ramp_end_index+1;i< best_path[0].size();++i)
			{
				v_points(i) = desired_speed;
			}
		}
		else if(lead_vehicle_v < start_speed)
		{
//			double desired_a = (start_speed*start_speed-desired_speed*desired_speed)/(2*(disToLead-vehicle_length_));
//			LOG(INFO) <<"desired acc: "<<desired_a;
//			double decel_distance = start_speed*time_gap + 1.0/2*(-a_max_)*time_gap*time_gap;
			int i = 0;
			double dist = 0.0;
			v_points(i) = start_speed;
			while((i < best_path[0].size()-1))
			{
				dist += planner_helper::distance(best_path[0](i+1)-best_path[0](i),
												 best_path[1](i+1)-best_path[1](i));
				double vf = calc_final_speed(start_speed, -a_max_, dist);
				if(vf < desired_speed)
					vf = desired_speed;
				i++;
				v_points(i) = vf;
			}

			for(int j = i+1;j< best_path[0].size();++j)
			{
				v_points(j) = desired_speed;
			}

		}*/

		std::cout<<"ramp_end_index before: "<<ramp_end_index<<std::endl;
		double distance = disToLead-planner_helper::distance(best_path[0](ramp_end_index)-best_path[0](0),best_path[1](ramp_end_index)-best_path[1](0));

//		if(std::abs(start_speed-lead_vehicle_v) > 0.1)
		{
			double distance_gap = desired_speed*time_gap_;
//			double distance_gap = start_speed*(disToLead - vehicle_length_)/(start_speed-lead_vehicle_v);
			LOG(INFO) <<"distance　gap: "<<distance_gap;
			while((ramp_end_index > 0) && (distance < distance_gap))
			{
				distance += planner_helper::distance(best_path[0](ramp_end_index)-best_path[0](ramp_end_index-1),
													 best_path[1](ramp_end_index)-best_path[1](ramp_end_index-1));
				ramp_end_index--;
			}
		}
//		ramp_end_index--;
		std::cout<<"ramp_end_index after: "<<ramp_end_index<<std::endl;
//		double vi = start_speed;
		for(int i = 0;i<ramp_end_index + 1;++i)
		{
			double dist = planner_helper::distance(best_path[0](i+1)-best_path[0](i),
					 	 	 	 	 	 	 	   best_path[1](i+1)-best_path[1](i));
			double vf;
			if(desired_speed < start_speed)
			{
				vf = calc_final_speed(vi, -a_max_, dist);
				if(vf < desired_speed)
					vf = desired_speed;
			}
			else
			{
				vf = calc_final_speed(vi, a_max_, dist);
				if(vf > desired_speed)
					vf = desired_speed;
			}
			v_points(i) = vi;
			vi = vf;
		}
		for(int i = ramp_end_index+1;i< best_path[0].size();++i)
		{
			v_points(i) = desired_speed;
		}
		best_path.insert(best_path.end()-1,v_points);

		return;
	}
	void VelocityPlanner::nominal_profile(std::vector<Eigen::VectorXd>& best_path,double& start_speed,double& desired_speed)
	{
		Eigen::VectorXd v_points(best_path[0].size());
		if(best_path[0].size() == 0)
		{
			best_path.insert(best_path.end()-1,v_points);
			return;
		}
		double accel_distance;
		double a;
		int row,col;
		double kappa_max = (best_path[3].array().abs()).matrix().maxCoeff(&row,&col);
        /*if(kappa_max > 0.1)
        	LOG(INFO) <<"max　kappa: "<<kappa_max;*/
		double v_kappa_max;
		if(kappa_max < 0.05)
			v_kappa_max = std::numeric_limits<double>::max();
		else
//			v_kappa_max = std::sqrt(a_lat_max_/kappa_max);
			v_kappa_max = std::sqrt(a_lat_max_*minimum_turning_radius_);

		if(desired_speed < v_kappa_max)
		{
			if(desired_speed < start_speed)
				accel_distance = calc_distance(start_speed, desired_speed, -a_max_);
			else
				accel_distance = calc_distance(start_speed, desired_speed, a_max_);

			int ramp_end_index = 0;
			double distance = 0.0;
			while((ramp_end_index < best_path[0].size()-1) && (distance < accel_distance))
			{
				distance += planner_helper::distance(best_path[0](ramp_end_index+1)-best_path[0](ramp_end_index),
													 best_path[1](ramp_end_index+1)-best_path[1](ramp_end_index));
				ramp_end_index++;
			}

			if(ramp_end_index < (best_path[0].size() - 1))
			{
				while((ramp_end_index < best_path[0].size()-1))
				{
					distance += planner_helper::distance(best_path[0](ramp_end_index+1)-best_path[0](ramp_end_index),
														 best_path[1](ramp_end_index+1)-best_path[1](ramp_end_index));
					ramp_end_index++;
				}

				a = calc_acceleration(start_speed,desired_speed,distance);

			}
			else
				a = desired_speed < start_speed ? -a_max_ : a_max_;

			double vi = start_speed;
			for(int i = 0;i < ramp_end_index;++i)
			{
				double dist = planner_helper::distance(best_path[0](i+1)-best_path[0](i),
													   best_path[1](i+1)-best_path[1](i));
				double vf;
				if(desired_speed < start_speed)
				{
					vf = calc_final_speed(vi, a, dist);
					if(vf < desired_speed)
						vf = desired_speed;
				}
				else
				{
					vf = calc_final_speed(vi, a, dist);
					if(vf > desired_speed)
						vf = desired_speed;
				}
				v_points(i) = vi;
				vi = vf;
			}
			for(int i = ramp_end_index;i< best_path[0].size();++i)
			{
				v_points(i) = desired_speed;
			}
		}
		else
		{
			double vi = start_speed;
			for(int i = 0;i < row;++i)
			{
				double dist = planner_helper::distance(best_path[0](i+1)-best_path[0](i),
													   best_path[1](i+1)-best_path[1](i));
				double vf;
				if(v_kappa_max < start_speed)
				{
					vf = calc_final_speed(vi, -a_max_, dist);
					if(vf < v_kappa_max)
						vf = v_kappa_max;
				}
				else
				{
					vf = calc_final_speed(vi, a_max_, dist);
					if(vf > v_kappa_max)
						vf = v_kappa_max;
				}
				v_points(i) = vi;
				vi = vf;
			}
			for(int i = row;i< best_path[0].size();++i)
			{
				v_points(i) = v_kappa_max;
			}
			/*std::cout<<"v: ";
			for(int i = 0;i < v_points.size();++i)
				std::cout<<v_points(i)<<" ";
			std::cout<<std::endl;*/
		}

		best_path.insert(best_path.end()-1,v_points);
		return;
	}
	void VelocityPlanner::emergency_stop_profile(std::vector<Eigen::VectorXd>& best_path,double& start_speed)
	{
		Eigen::VectorXd v_points(best_path[0].size());
//		v_points(0) = start_speed;
		for(int i = 0;i < best_path[0].size();++i)
			v_points(i) = 0.0;
		best_path.insert(best_path.end()-1,v_points);

		return;
	}
	void VelocityPlanner::stop_wait_profile(std::vector<Eigen::VectorXd>& best_path,double& start_speed)
	{
		Eigen::VectorXd v_points(best_path[0].size());
//		v_points(0) = start_speed;
		for(int i = 0;i < best_path[0].size();++i)
			v_points(i) = 0.0;
		best_path.insert(best_path.end()-1,v_points);

		return;
	}

	void VelocityPlanner::compute_velocity_profile(std::vector<Eigen::VectorXd>& best_path,double& desired_speed,EgoState& ego_state,StateType& current_behaviour,my_nav_msgs::DetectedObject& lead_vehicle)
	{
		double start_speed = ego_state.open_loop_speed;

		if(current_behaviour == state::DECELERATE_TO_STOP)
		{
			decelerate_profile(best_path, start_speed);
		}
		else if(current_behaviour == state::FOLLOW_LEAD_VEHICLE)
		{
			follow_profile(best_path, start_speed, desired_speed,lead_vehicle);
		}
		else if(current_behaviour == state::FOLLOW_LANE || current_behaviour == state::BACK)
		{
			/*if(current_behaviour == state::BACK)
				desired_speed = 1.0;*/
			nominal_profile(best_path, start_speed, desired_speed);
		}
		else if(current_behaviour == state::AVOID_OBSTACLE)
		{
			desired_speed = std::min(obstacle_vel_,desired_speed);
			nominal_profile(best_path, start_speed, desired_speed);
		}
		else if(current_behaviour == state::EMERGENCY_STOP)
		{
			emergency_stop_profile(best_path,start_speed);
		}
		else if(current_behaviour == state::STOP_WAIT)
		{
			stop_wait_profile(best_path,start_speed);
		}

		if(best_path[0].size() > 1)
		{
			best_path[0](0) = (best_path[0](1)-best_path[0](0))*0.1 + best_path[0](0);
			best_path[1](0) = (best_path[1](1)-best_path[1](0))*0.1 + best_path[1](0);
			best_path[2](0) = (best_path[2](1)-best_path[2](0))*0.1 + best_path[2](0);
			best_path[3](0) = (best_path[3](1)-best_path[3](0))*0.1 + best_path[3](0);
			best_path[4](0) = (best_path[4](1)-best_path[4](0))*0.1 + best_path[4](0);
		}
		prev_trajectory_.waypoints.clear();
		for(int i = 0;i < best_path[0].size();++i)
		{
			my_nav_msgs::Waypoint point;
			point.pose.pose.position.x = best_path[0](i);
			point.pose.pose.position.y = best_path[1](i);
			point.twist.twist.linear.x = best_path[3](i);
			prev_trajectory_.waypoints.push_back(point);
		}

		if(current_behaviour == state::BACK)
		{
			for(int i = 0;i < best_path[0].size();++i)
			{
				best_path[3](i) = -best_path[3](i);
			}
		}

		return;
	}
}


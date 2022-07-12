/*
 * path_generation.cpp
 *
 *  Created on: Feb 18, 2020
 *      Author: rcj
 */


#include "conformal_lattice_planner/path_planning/path_generation.h"


namespace planning
{
	PathOptimizer::PathOptimizer(apollo::common::VehicleParam& vehicle_param,ConformalLatticeConfig& conformal_lattice_config)

	{
		vehicle_param_ = vehicle_param;
		conformal_lattice_config_ = conformal_lattice_config;

		minimum_turning_radius_ = vehicle_param_.min_turn_radius();
		num_points_ = conformal_lattice_config_.path_planning_config().num_points();
	}
	PathOptimizer::~PathOptimizer()
	{

	}
	Eigen::VectorXd  PathOptimizer::kappaS(double& a,double& b,double& c,double& d,Eigen::VectorXd& s)
	{
		Eigen::VectorXd kappas = b*s + c*(s.array().square()).matrix() + d*(s.array().cube()).matrix();
		return kappas;
	}
	Eigen::VectorXd  PathOptimizer::thetaS(double& a,double& b,double& c,double& d,Eigen::VectorXd& s)
	{
		Eigen::VectorXd thetas = a*s+(b/2)*(s.array().square()).matrix() + (c/3)*(s.array().cube()).matrix() + (d/4)*(s.array().pow(4)).matrix();
		return thetas;
	}

	Eigen::VectorXd PathOptimizer::xS(Eigen::VectorXd& thetas,Eigen::VectorXd& s)
	{
		Eigen::VectorXd xs(s.size());
		xs(0) = 0.0;
		for(int i = 0;i<s.size()-1;++i)
		{
			double tmp = (std::cos(thetas(i+1))+std::cos(thetas(i)))*(s(i+1)-s(i))/2;
			xs(i+1) = xs(i) +tmp;
		}
		return xs;
	}
	Eigen::VectorXd PathOptimizer::yS(Eigen::VectorXd& thetas,Eigen::VectorXd& s)
	{
		Eigen::VectorXd ys(s.size());
		ys(0) = 0.0;
		for(int i = 0;i<s.size()-1;++i)
		{
			double tmp = (std::sin(thetas(i+1))+std::sin(thetas(i)))*(s(i+1)-s(i))/2;
			ys(i+1) = ys(i) +tmp;
		}
		return ys;
	}

	std::vector<Eigen::VectorXd> PathOptimizer::sample_spiral(std::vector<double>& solution)
	{
		std::vector<double> p{0.0,solution[0],solution[1],0.0,solution[2]};
        double a = p[0];
        double b = -(11.0*p[0]/2.0 - 9.0*p[1] + 9.0*p[2]/2.0 - p[3])/p[4];
        double c = (9.0*p[0] - 45.0*p[1]/2.0 + 18.0*p[2] - 9.0*p[3]/2.0)/(p[4]*p[4]);
        double d = -(9.0*p[0]/2.0 - 27.0*p[1]/2.0 + 27.0*p[2]/2.0 - 9.0*p[3]/2.0)/(p[4]*p[4]*p[4]);

        Eigen::VectorXd s_points;
        s_points.setLinSpaced(num_points_,0.0,p[4]);

        Eigen::VectorXd kappa_points = kappaS(a, b, c, d, s_points);
        Eigen::VectorXd t_points = thetaS(a, b, c, d, s_points);
        Eigen::VectorXd x_points = xS(t_points,s_points);
        Eigen::VectorXd y_points = yS(t_points,s_points);

        return std::vector<Eigen::VectorXd>{x_points,y_points,t_points,kappa_points};
	}

	std::vector<Eigen::VectorXd> PathOptimizer::optimize_spiral(std::vector<double>& goal_state,StateType& current_behaviour)
	{
		xf_ = goal_state[0];
		yf_ = goal_state[1];
		tf_ = goal_state[2];

		double sf_0 = std::sqrt(xf_*xf_+yf_*yf_);

		typedef CPPAD_TESTVECTOR(double) Dvector;

		int n_p = 3;

		Dvector pi(n_p);

		pi[0] = 0.0;
		pi[1] = 0.0;
		pi[2] = current_behaviour == state::BACK ? -sf_0 : sf_0;

		Dvector pl(n_p), pu(n_p);
		for(int i = 0; i < n_p-1; i++)
		{   pl[i] = -1.0/2;//minimum_turning_radius_;
			pu[i] = 1.0/2;//minimum_turning_radius_;
		}
		pl[2] = current_behaviour == state::BACK ? -std::numeric_limits<double>::max() : sf_0;
		pu[2] = current_behaviour == state::BACK ? -sf_0 : std::numeric_limits<double>::max();

		Dvector gl,gu;

		FG_eval fg_eval;

		// options
		std::string options;
		// comment this if you'd like more print information ==turn off any printing
		options += "Integer print_level  0\n";
		/*options += "String  sb           yes\n";
		// maximum number of iterations
		options += "Integer max_iter     1000\n";
		// approximate accuracy in first order necessary conditions;
		// see Mathematical Programming, Volume 106, Number 1,
		// Pages 25-57, Equation (6)
		options += "Numeric tol          1e-6\n";
		// derivative testing
		options += "String  derivative_test            second-order\n";
		// maximum amount of random pertubation; e.g.,
		// when evaluation finite diff
		options += "Numeric point_perturbation_radius  0.\n";*/
		// NOTE: Setting sparse to true allows the solver to take advantage
		// of sparse routines, this makes the computation MUCH FASTER. If you
		// can uncomment 1 of these and see if it makes a difference or not but
		// if you uncomment both the computation time should go up in orders of
		// magnitude.
		options += "Sparse  true        forward\n";
		options += "Sparse  true        reverse\n";
		// NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
		// Change this as you see fit.
		options += "Numeric max_cpu_time          0.5\n";

		// place to return solution
		CppAD::ipopt::solve_result<Dvector> solution;

		// solve the problem
		CppAD::ipopt::solve<Dvector, FG_eval>(
			options, pi, pl, pu, gl, gu, fg_eval, solution
		);

		std::vector<Eigen::VectorXd> spiral;
		if(solution.status == CppAD::ipopt::solve_result<Dvector>::success)
		{
			std::vector<double> p{solution.x[0],solution.x[1],solution.x[2]};

			spiral = sample_spiral(p);
		}
		else
		{
			std::cout<<"optimization failed!!"<<std::endl;
		}
		return spiral;
	}

CollisionChecker::CollisionChecker(const apollo::common::VehicleParam& vehicle_param)
	:vehicle_param_(vehicle_param)
	{
		wheel_base_ = vehicle_param_.wheel_base();
		vehicle_width_ = vehicle_param_.width();
		vehicle_length_ = vehicle_param_.length();

		front_edge_to_center_ = vehicle_param_.front_edge_to_center();
		back_edge_to_center_ = vehicle_param_.back_edge_to_center();
		left_edge_to_center_ = vehicle_param_.left_edge_to_center();
		right_edge_to_center_ = vehicle_param_.right_edge_to_center();

		double half_width = vehicle_width_/2.0;
		double radius = std::sqrt(back_edge_to_center_*back_edge_to_center_ + half_width*half_width);

		CIRCLE_OFFSETS_ = std::vector<double>{0.0,wheel_base_/2.0,wheel_base_};
		CIRCLE_RADII_ = std::vector<double>(3,radius);
	}
	CollisionChecker::~CollisionChecker()
	{

	}
	bool CollisionChecker::is_collision(const my_nav_msgs::DetectedObject& object,const Eigen::MatrixXd& circle_locations)
	{
		std::cout<<"obstacle points: "<<object.convex_hull.polygon.points.size()<<std::endl;
		for(int i = 0;i<circle_locations.rows();++i)
		{
			for(auto p : object.convex_hull.polygon.points)
			{
				Eigen::VectorXd diff(2);
				diff<<p.x-circle_locations(i,0),p.y-circle_locations(i,1);
				if(diff.norm() < CIRCLE_RADII_[i])
					return true;
			}
		}
		return false;
	}
	bool CollisionChecker::isSameLaneDynamicObject(my_nav_msgs::DetectedObject object)
	{
		static double cos_value = vehicle_length_/std::sqrt(vehicle_length_*vehicle_length_+(lane_width_/2.0)*(lane_width_/2.0));
		double object_x = object.pose.position.x;
		double object_y = object.pose.position.y;
		double object_v = object.velocity.linear.x;

		double deta_x = object_x - 0.0;
		double deta_y = object_y - 0.0;

		double object_distance = std::sqrt(std::pow(deta_x,2)+std::pow(deta_y,2));
		deta_x/=object_distance;
		deta_y/=object_distance;

		double ego_heading_vector_x = std::cos(0.0);
		double ego_heading_vector_y = std::sin(0.0);

		if((deta_x*ego_heading_vector_x+deta_y*ego_heading_vector_y) < cos_value || std::abs(object_y) > (lane_width_/2.0))
			return false;
		if(std::abs(object_v) < 0.1)
			return false;

		return true;
	}
	std::vector<bool> CollisionChecker::collision_check(std::vector<std::vector<Eigen::VectorXd>>& paths,my_nav_msgs::DetectedObjectArray& tracked_objects)
	{
		std::vector<bool> collision_check_array(paths.size(),true);

		if(tracked_objects.objects.size() == 0)
			return collision_check_array;

		for(int i =0;i<paths.size();++i)
		{
			bool collision_free = true;
			std::vector<Eigen::VectorXd> path = paths[i];

			for(int j = 0;j<path[0].size();++j)
			{
				Eigen::MatrixXd circle_locations = Eigen::MatrixXd::Zero(CIRCLE_OFFSETS_.size(),2);

				for(int k = 0;k<CIRCLE_OFFSETS_.size();++k)
				{
					circle_locations(k,0) = path[0](j) + CIRCLE_OFFSETS_[k]*std::cos(path[2](j));
					circle_locations(k,1) = path[1](j) + CIRCLE_OFFSETS_[k]*std::sin(path[2](j));
				}

				for(auto object : tracked_objects.objects)
				{
					/*if(isSameLaneDynamicObject(object))
						continue;*/
					if(is_collision(object,circle_locations))
					{
						collision_free =false;
						break;
					}
				}
				if(!collision_free)
					break;
			}

			collision_check_array[i] = collision_free;
		}
		return collision_check_array;
	}



}

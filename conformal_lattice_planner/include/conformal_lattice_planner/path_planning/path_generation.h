/*
 * path_generation.h
 *
 *  Created on: Feb 18, 2020
 *      Author: rcj
 */

#pragma once

#include <vector>
#include <cmath>
#include <limits>

#include "my_nav_msgs/DetectedObjectArray.h"

#include <cppad/ipopt/solve.hpp>
#include <cppad/cppad.hpp>

#include <Eigen/Dense>

#include "conformal_lattice_config.pb.h"
#include "vehicle_config.pb.h"

#include "planner_helper/planner_helper.h"

static double xf_ = 0.0;
static double yf_ = 0.0;
static double tf_ = 0.0;//yaw_f

namespace planning
{
	class PathOptimizer
	{
	public:

		PathOptimizer(apollo::common::VehicleParam& vehicle_param,ConformalLatticeConfig& conformal_lattice_config);
		~PathOptimizer();
		std::vector<Eigen::VectorXd> optimize_spiral(std::vector<double>& goal_state,StateType& current_behaviour);
	private:
		double minimum_turning_radius_;
		int num_points_;

		apollo::common::VehicleParam vehicle_param_;
		ConformalLatticeConfig conformal_lattice_config_;

		std::vector<Eigen::VectorXd> sample_spiral(std::vector<double>& solution);
		Eigen::VectorXd kappaS(double& a,double& b,double& c,double& d,Eigen::VectorXd& s);
		Eigen::VectorXd thetaS(double& a,double& b,double& c,double& d,Eigen::VectorXd& s);
		Eigen::VectorXd xS(Eigen::VectorXd& thetas,Eigen::VectorXd& s);
		Eigen::VectorXd yS(Eigen::VectorXd& thetas,Eigen::VectorXd& s);
	};

	class CollisionChecker
	{
	public:
		explicit CollisionChecker(const apollo::common::VehicleParam& vehicle_param);
		~CollisionChecker();

		std::vector<bool> collision_check(std::vector<std::vector<Eigen::VectorXd>>& paths,my_nav_msgs::DetectedObjectArray& tracked_objects);

	private:
		std::vector<double> CIRCLE_OFFSETS_;
		std::vector<double> CIRCLE_RADII_;
		int path_select_weight;

		apollo::common::VehicleParam vehicle_param_;
		double lane_width_;

		double vehicle_length_;
		double vehicle_width_;
		double front_edge_to_center_;
		double back_edge_to_center_;
		double left_edge_to_center_;
		double right_edge_to_center_;
		double wheel_base_;		

		bool is_collision(const my_nav_msgs::DetectedObject& object,const Eigen::MatrixXd& circle_locations);
		bool isSameLaneDynamicObject(my_nav_msgs::DetectedObject object);
	};

}

namespace {
    using CppAD::AD;

    class FG_eval
    {
    public:
        typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;
        void operator()(ADvector& fg, const ADvector& x)
        {   assert( fg.size() == 1 );
            assert( x.size()  == 3 );

            ADvector p(5);
            p[0] = 0.0;
            p[1] = x[0];
            p[2] = x[1];
            p[3] = 0.0;
            p[4] = x[2];

            // f(x)
            fg[0] = fbe(p)+25*(fxf(p) + fyf(p)) + 30*ftf(p);
            //
            return;
        }

        AD<double>fbe(ADvector& p)
		{
        	AD<double> t0 = p[4]*(p[0]*p[1]*9.9E1-p[0]*p[2]*3.6E1+p[0]*p[3]*1.9E1-p[1]*p[2]*8.1E1-p[1]*p[3]*3.6E1+p[2]*p[3]*9.9E1+(p[0]*p[0])*6.4E1+(p[1]*p[1])*3.24E2+(p[2]*p[2])*3.24E2+(p[3]*p[3])*6.4E1)*(1.0/8.4E2);
        	return t0;
		}
        AD<double> fxf(ADvector& p)
		{
        	AD<double> t2 = p[0]*(1.1E1/2.0);
        	AD<double> t3 = p[1]*9.0;
        	AD<double> t4 = p[2]*(9.0/2.0);
        	AD<double> t5 = p[0]*(9.0/2.0);
        	AD<double> t6 = p[1]*(2.7E1/2.0);
        	AD<double> t7 = p[2]*(2.7E1/2.0);
        	AD<double> t8 = p[3]*(9.0/2.0);
        	AD<double> t9 = t5-t6+t7-t8;
        	AD<double> t10 = p[0]*9.0;
        	AD<double> t11 = p[1]*(4.5E1/2.0);
        	AD<double> t12 = p[2]*1.8E1;
        	AD<double> t13 = t8-t10+t11-t12;
        	AD<double> t14 = p[3]-t2+t3-t4;
        	AD<double> t15 = xf_-p[4]*(CppAD::cos(p[0]*p[4]-p[4]*t9*(1.0/4.0)-p[4]*t13*(1.0/3.0)+p[4]*t14*(1.0/2.0))+CppAD::cos(p[0]*p[4]*(1.0/2.0)-p[4]*t9*(1.0/6.4E1)-p[4]*t13*(1.0/2.4E1)+p[4]*t14*(1.0/8.0))*2.0+CppAD::cos(p[0]*p[4]*(3.0/4.0)-p[4]*t9*7.91015625E-2-p[4]*t13*(9.0/6.4E1)+p[4]*t14*(9.0/3.2E1))*2.0+CppAD::cos(p[0]*p[4]*(1.0/4.0)-p[4]*t9*9.765625E-4-p[4]*t13*(1.0/1.92E2)+p[4]*t14*(1.0/3.2E1))*2.0+CppAD::cos(p[0]*p[4]*(3.0/8.0)-p[4]*t9*4.94384765625E-3-p[4]*t13*(9.0/5.12E2)+p[4]*t14*(9.0/1.28E2))*4.0+CppAD::cos(p[0]*p[4]*(1.0/8.0)-p[4]*t9*6.103515625E-5-p[4]*t13*6.510416666666667E-4+p[4]*t14*(1.0/1.28E2))*4.0+CppAD::cos(p[0]*p[4]*(5.0/8.0)-p[4]*t9*3.814697265625E-2-p[4]*t13*8.138020833333333E-2+p[4]*t14*(2.5E1/1.28E2))*4.0+CppAD::cos(p[0]*p[4]*(7.0/8.0)-p[4]*t9*1.4654541015625E-1-p[4]*t13*2.233072916666667E-1+p[4]*t14*(4.9E1/1.28E2))*4.0+1.0)*(1.0/2.4E1);
        	AD<double> t0 = t15*t15;
			return t0;
		}
        AD<double> fyf(ADvector& p)
		{
        	AD<double> t2 = p[0]*(1.1E1/2.0);
        	AD<double> t3 = p[1]*9.0;
        	AD<double> t4 = p[2]*(9.0/2.0);
        	AD<double> t5 = p[0]*(9.0/2.0);
        	AD<double> t6 = p[1]*(2.7E1/2.0);
        	AD<double> t7 = p[2]*(2.7E1/2.0);
        	AD<double> t8 = p[3]*(9.0/2.0);
        	AD<double> t9 = t5-t6+t7-t8;
        	AD<double> t10 = p[0]*9.0;
        	AD<double> t11 = p[1]*(4.5E1/2.0);
        	AD<double> t12 = p[2]*1.8E1;
        	AD<double> t13 = t8-t10+t11-t12;
        	AD<double> t14 = p[3]-t2+t3-t4;
        	AD<double> t15 = yf_-p[4]*(CppAD::sin(p[0]*p[4]-p[4]*t9*(1.0/4.0)-p[4]*t13*(1.0/3.0)+p[4]*t14*(1.0/2.0))+CppAD::sin(p[0]*p[4]*(1.0/2.0)-p[4]*t9*(1.0/6.4E1)-p[4]*t13*(1.0/2.4E1)+p[4]*t14*(1.0/8.0))*2.0+CppAD::sin(p[0]*p[4]*(3.0/4.0)-p[4]*t9*7.91015625E-2-p[4]*t13*(9.0/6.4E1)+p[4]*t14*(9.0/3.2E1))*2.0+CppAD::sin(p[0]*p[4]*(1.0/4.0)-p[4]*t9*9.765625E-4-p[4]*t13*(1.0/1.92E2)+p[4]*t14*(1.0/3.2E1))*2.0+CppAD::sin(p[0]*p[4]*(3.0/8.0)-p[4]*t9*4.94384765625E-3-p[4]*t13*(9.0/5.12E2)+p[4]*t14*(9.0/1.28E2))*4.0+CppAD::sin(p[0]*p[4]*(1.0/8.0)-p[4]*t9*6.103515625E-5-p[4]*t13*6.510416666666667E-4+p[4]*t14*(1.0/1.28E2))*4.0+CppAD::sin(p[0]*p[4]*(5.0/8.0)-p[4]*t9*3.814697265625E-2-p[4]*t13*8.138020833333333E-2+p[4]*t14*(2.5E1/1.28E2))*4.0+CppAD::sin(p[0]*p[4]*(7.0/8.0)-p[4]*t9*1.4654541015625E-1-p[4]*t13*2.233072916666667E-1+p[4]*t14*(4.9E1/1.28E2))*4.0)*(1.0/2.4E1);
        	AD<double> t0 = t15*t15;
			return t0;
		}
        AD<double> ftf(ADvector& p)
		{
        	AD<double> t2 = tf_-p[0]*p[4]+p[4]*(p[0]*(1.1E1/2.0)-p[1]*9.0+p[2]*(9.0/2.0)-p[3])*(1.0/2.0)+p[4]*(p[0]*(9.0/2.0)-p[1]*(2.7E1/2.0)+p[2]*(2.7E1/2.0)-p[3]*(9.0/2.0))*(1.0/4.0)-p[4]*(p[0]*9.0-p[1]*(4.5E1/2.0)+p[2]*1.8E1-p[3]*(9.0/2.0))*(1.0/3.0);
        	AD<double> t0 = t2*t2;
			return t0;
		}

    };
}



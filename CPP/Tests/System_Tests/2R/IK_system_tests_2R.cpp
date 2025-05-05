#include <iostream>
#include <chrono>
#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>

// Hacky way to access private class members for testing purposes
// Do not use this in anything but these system tests
#define private   public
#define protected public

#include "EAIK.h"

// Hacky way to access private class members for testing purposes
// Do not use this in anything but these system tests
#undef private
#undef protected

#include "IK_test_utils.h"
#include "IK_system_tests_2R.h"

//
// 2R Tests
//

bool run_2R_Tests()
{
	bool allPass = true;
	allPass &= ik_test_2R_pos_not_intersecting_parallel();
	allPass &= ik_test_2R_pos_12_parallel();
	allPass &= ik_test_2R_pos_12_intersecting();
	allPass &= ik_test_2R_orientation_not_intersecting_parallel();
	
	// 2R Tests
	std::cout<< std::endl<<"====================== RESULT: ======================"<<std::endl;

	if(allPass)
	{
		std::cout<< "                     2R  PASSING                     "<<std::endl;
	}
	else
	{
		std::cout<< "                     2R   FAILED                     "<<std::endl;
	}

	std::cout<< "====================================================="<<std::endl;

	return allPass;
}

bool ik_test_2R_pos_12_intersecting()
{
	Eigen::Matrix<double, 3, 2> H;
	H << ez, ex;
	Eigen::Matrix<double, 3, 3> P;
	P << ez, ex, ey;

	EAIK::Robot two_parallel(H, P);
	
	std::vector<Eigen::Vector3d> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(two_parallel.fwdkin(std::vector{rand_angle(), rand_angle()}).block<3, 1>(0, 3));
	}

	return evaluate_test_position<IKS::General_2R>("IK 2R position - 1,2 Intersecting", *dynamic_cast<IKS::General_2R*>(two_parallel.bot_kinematics.get()), ee_poses);
}

bool ik_test_2R_pos_12_parallel()
{
	Eigen::Matrix<double, 3, 2> H;
	H << ez, ez;
	Eigen::Matrix<double, 3, 3> P;
	P << ez, ex, ey;

	EAIK::Robot two_parallel(H, P);
	
	std::vector<Eigen::Vector3d> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(two_parallel.fwdkin(std::vector{rand_angle(), rand_angle()}).block<3, 1>(0, 3));
	}

	return evaluate_test_position<IKS::General_2R>("IK 2R position - 1,2 Parallel", *dynamic_cast<IKS::General_2R*>(two_parallel.bot_kinematics.get()), ee_poses);
}

bool ik_test_2R_pos_not_intersecting_parallel()
{
	Eigen::Matrix<double, 3, 2> H;
	H << ez, ex;
	Eigen::Matrix<double, 3, 3> P;
	P << ez, ex+ey, ey;

	EAIK::Robot two_parallel(H, P);
	
	std::vector<Eigen::Vector3d> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(two_parallel.fwdkin(std::vector{rand_angle(), rand_angle()}).block<3, 1>(0, 3));
	}

	return evaluate_test_position<IKS::General_2R>("IK 2R position - no axes intersecting or parallel", *dynamic_cast<IKS::General_2R*>(two_parallel.bot_kinematics.get()), ee_poses);
}

bool ik_test_2R_orientation_not_intersecting_parallel()
{
	Eigen::Matrix<double, 3, 2> H;
	H << ez, ex;
	Eigen::Matrix<double, 3, 3> P;
	P << ez, ex+ey, ey;

	EAIK::Robot two_parallel(H, P);
	
	std::vector<Eigen::Matrix3d> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(two_parallel.fwdkin(std::vector{rand_angle(), rand_angle()}).block<3, 3>(0, 0));
	}
	
	return evaluate_test_orientation<IKS::General_2R>("IK 2R orientation - no axes intersecting or parallel", *dynamic_cast<IKS::General_2R*>(two_parallel.bot_kinematics.get()), ee_poses);
}

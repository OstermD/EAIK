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
#include "IK_system_tests_1R.h"

//
// 1R Tests
//

bool run_1R_Tests()
{
	bool allPass = true;
	allPass &= ik_test_1R_pos();
	allPass &= ik_test_1R_orientation();

	// 1R Tests
	std::cout<< std::endl<<"====================== RESULT: ======================"<<std::endl;

	if(allPass)
	{
		std::cout<< "                     1R  PASSING                     "<<std::endl;
	}
	else
	{
		std::cout<< "                     1R   FAILED                     "<<std::endl;
	}

	std::cout<< "====================================================="<<std::endl;

	return allPass;
}

bool ik_test_1R_pos()
{
	Eigen::Matrix<double, 3, 1> H;
	H << ez;
	Eigen::Matrix<double, 3, 2> P;
	P << ez, ex;

	EAIK::Robot two_parallel(H, P);
	
	std::vector<Eigen::Vector3d> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(two_parallel.fwdkin(std::vector{rand_angle()}).block<3, 1>(0, 3));
	}

	return evaluate_test_position<IKS::General_1R>("IK 1R position", *dynamic_cast<IKS::General_1R*>(two_parallel.bot_kinematics.get()), ee_poses);
}

bool ik_test_1R_orientation()
{
	Eigen::Matrix<double, 3, 1> H;
	H << ez;
	Eigen::Matrix<double, 3, 2> P;
	P << ez, ex;

	EAIK::Robot two_parallel(H, P);
	
	std::vector<Eigen::Matrix3d> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(two_parallel.fwdkin(std::vector{rand_angle()}).block<3, 3>(0, 0));
	}

	return evaluate_test_orientation<IKS::General_1R>("IK 1R orientation", *dynamic_cast<IKS::General_1R*>(two_parallel.bot_kinematics.get()), ee_poses);
}
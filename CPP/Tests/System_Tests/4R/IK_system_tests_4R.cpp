#include <iostream>
#include <chrono>
#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "EAIK.h"
#include "IK_test_utils.h"
#include "IK_system_tests_4R.h"

//
// 4R Tests
//

bool run_4R_Tests()
{
	bool allPass = true;
	allPass &= ik_test_4R_non_intersecting_parallel();

	// Intersecting axes
	allPass &= ik_test_4R_34_intersecting();
	allPass &= ik_test_4R_12_intersecting();
	allPass &= ik_test_4R_23_intersecting();
	allPass &= ik_test_4R_spherical_wrist();
	allPass &= ik_test_4R_spherical_base();
	allPass &= ik_test_4R_12_34_intersecting();

	// Parallels axes
	allPass &= ik_test_4R_12_parallel();
	allPass &= ik_test_4R_34_parallel();
	allPass &= ik_test_4R_23_parallel();
	allPass &= ik_test_4R_123_parallel();
	allPass &= ik_test_4R_234_parallel();
	
	std::cout<< std::endl<<"====================== RESULT: ======================"<<std::endl;

	if(allPass)
	{
		std::cout<< "                     4R  PASSING                     "<<std::endl;
	}
	else
	{
		std::cout<< "                     4R   FAILED                     "<<std::endl;
	}

	std::cout<< "====================================================="<<std::endl;

	return allPass;
}

bool ik_test_4R_non_intersecting_parallel()
{
	Eigen::Matrix<double, 3, 4> H;
	H << ey, ez, ey, ex;
	Eigen::Matrix<double, 3, 5> P;
	P << zv, ex + ez, ex + ez, ex + ez, ez;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 4R no axes parallel or intersecting", robot, ee_poses);
}

bool ik_test_4R_34_intersecting()
{
	Eigen::Matrix<double, 3, 4> H;
	H << ez, ey, ex, ey;
	Eigen::Matrix<double, 3, 5> P;
	P << ex + ez, ex + ez, ex + ez, ex+ey, ez;
	
	EAIK::Robot robot(H, P);
	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 4R axes 3,4 intersecting", robot, ee_poses);
}

bool ik_test_4R_12_intersecting()
{
	Eigen::Matrix<double, 3, 4> H;
	H << ey, ex, ey, ez;
	Eigen::Matrix<double, 3, 5> P;
	P << ez, ey+ex, ex+ez, ex+ez, ex+ez;

	EAIK::Robot robot(H, P);
	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 4R axes 1,2 intersecting", robot, ee_poses);
}

bool ik_test_4R_23_intersecting()
{
	Eigen::Matrix<double, 3, 4> H;
	H << ez, ey, ex, ey;
	Eigen::Matrix<double, 3, 5> P;
	P << ex, ex+ey, ey+ex, ex+ez, ex+ez;

	EAIK::Robot robot(H, P);
	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 4R axes 2,3 intersecting", robot, ee_poses);
}

bool ik_test_4R_12_34_intersecting()
{
	Eigen::Matrix<double, 3, 4> H;
	H << ey, ex, ex, ey;
	Eigen::Matrix<double, 3, 5> P;
	P <<  ez, zv, ex + ez, zv, ez;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 4R axes 1,2 and 3,4 intersecting", robot, ee_poses);
}

bool ik_test_4R_spherical_base()
{
	Eigen::Matrix<double, 3, 4> H;
	H << ex, ey, ex, ey;
	Eigen::Matrix<double, 3, 5> P;
	P << zv, ex, zv, ex+ez, ex+ez;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 4R spherical base", robot, ee_poses);
}

bool ik_test_4R_spherical_wrist()
{
	Eigen::Matrix<double, 3, 4> H;
	H << ey, ex, ey, ex;
	Eigen::Matrix<double, 3, 5> P;
	P << ex + ez, ex + ez, zv, ex, zv;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 4R spherical wrist", robot, ee_poses);
}

bool ik_test_4R_12_parallel()
{
	Eigen::Matrix<double, 3, 4> H;
	H << ez, ez, ey, ex;
	Eigen::Matrix<double, 3, 5> P;
	P << zv, ex + ez, ex + ez, ex + ez, ez;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 4R axes 1,2 parallel", robot, ee_poses);
}

bool ik_test_4R_34_parallel()
{
	Eigen::Matrix<double, 3, 4> H;
	H << ey, ez, ey, ey;
	Eigen::Matrix<double, 3, 5> P;
	P << zv, ex + ez, ex + ez, ex + ez, ez;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 4R axes 3,4 parallel", robot, ee_poses);
}

bool ik_test_4R_23_parallel()
{
	Eigen::Matrix<double, 3, 4> H;
	H << ex, ey, ey, ex;
	Eigen::Matrix<double, 3, 5> P;
	P << zv, ey + ez, ex + ez, ex + ez, ez;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 4R axes 2,3 parallel", robot, ee_poses);
}	

bool ik_test_4R_123_parallel()
{
	Eigen::Matrix<double, 3, 4> H;
	H <<  ex, ex, ex, ez;
	Eigen::Matrix<double, 3, 5> P;
	P <<  ez, ey, ey, ey, ex+ey;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 4R axes 1,2,3 parallel", robot, ee_poses);
}

bool ik_test_4R_234_parallel()
{
	Eigen::Matrix<double, 3, 4> H;
	H <<  ez, ex, ex, ex;
	Eigen::Matrix<double, 3, 5> P;
	P <<  ez, ex+ey, ey, ey, ey;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 4R axes 2,3,4 parallel", robot, ee_poses);
}
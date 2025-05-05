#include <iostream>
#include <chrono>
#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "EAIK.h"
#include "IK_test_utils.h"
#include "IK_system_tests_5R.h"

//
// 5R Tests
//

bool run_5R_Tests()
{
	bool allPass = true;
	
	// Three Parallel
	allPass &= ik_test_5R_123_Parallel();
	allPass &= ik_test_5R_345_Parallel();
	allPass &= ik_test_5R_234_Parallel();
	allPass &= ik_test_5R_123_Parallel_45_Parallel();
	allPass &= ik_test_5R_345_Parallel_12_Parallel();
	allPass &= ik_test_5R_123_Parallel_34_intersecting();

	// Two intersecting
	allPass &= ik_test_5R_12_intersecting();
	allPass &= ik_test_5R_45_intersecting();
	allPass &= ik_test_5R_12_45_intersecting();
	allPass &= ik_test_5R_23_45_intersecting();
	allPass &= ik_test_5R_12_parallel_45_intersecting();
	allPass &= ik_test_5R_23_parallel_45_intersecting();
	allPass &= ik_test_5R_23_parallel_45_intersecting_2();

	// Three intersecting
	allPass &= ik_test_5R_spherical_base();
	allPass &= ik_test_5R_spherical_wrist();
	allPass &= ik_test_5R_spherical_wrist_12_intersecting();
	allPass &= ik_test_5R_spherical_base_45_intersecting();

	// Two intermediate intersecting - two parallels
	allPass &= ik_test_5R_34_intersecting_23_parallel();
	allPass &= ik_test_5R_23_intersecting_34_parallel();
	allPass &= ik_test_5R_34_intersecting_23_parallel_45_parallel();

	std::cout<< std::endl<<"====================== RESULT: ======================"<<std::endl;

	if(allPass)
	{
		std::cout<< "                     5R  PASSING                     "<<std::endl;
	}
	else
	{
		std::cout<< "                     5R   FAILED                     "<<std::endl;
	}

	std::cout<< "====================================================="<<std::endl;

	return allPass;
}

// 3 parallel axes

// Axis 1, 2, 3, parallel
bool ik_test_5R_123_Parallel()
{
	Eigen::Matrix<double, 3, 5> three_parallel_H;
	three_parallel_H <<  ex, ex, ex, ez, ey;
	Eigen::Matrix<double, 3, 6> three_parallel_P;
	three_parallel_P <<  ez, ey, ey, ey, ex+ey, ex;

	EAIK::Robot three_parallel(three_parallel_H, three_parallel_P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);

	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(three_parallel.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK three parallel - 1,2,3", three_parallel, ee_poses);
}

bool ik_test_5R_123_Parallel_34_intersecting()
{
	Eigen::Matrix<double, 3, 5> three_parallel_H;
	three_parallel_H <<  ex, ex, ex, ez, ey;
	Eigen::Matrix<double, 3, 6> three_parallel_P;
	three_parallel_P <<  ez, ey, ey, ex+ez, ex+ey, ex;

	EAIK::Robot three_parallel(three_parallel_H, three_parallel_P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);

	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(three_parallel.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK three parallel - 1,2,3 and 3,4 intersecting", three_parallel, ee_poses);
}


bool ik_test_5R_123_Parallel_45_Parallel()
{
	Eigen::Matrix<double, 3, 5> three_parallel_H;
	three_parallel_H <<  ex, ex, ex, ez, ez;
	Eigen::Matrix<double, 3, 6> three_parallel_P;
	three_parallel_P <<  ez, ey, ey, ey, ex+ey, ex;

	EAIK::Robot three_parallel(three_parallel_H, three_parallel_P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);

	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(three_parallel.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK three parallel - 1,2,3 and 4,5 Parallel", three_parallel, ee_poses);
}

// Axis 3, 4, 5, parallel
bool ik_test_5R_345_Parallel()
{
	Eigen::Matrix<double, 3, 5> three_parallel_H;
	three_parallel_H <<  ex, ez, ex, ex, ex;
	Eigen::Matrix<double, 3, 6> three_parallel_P;
	three_parallel_P <<  ex, ex+ey, ey, ey, ey, ez;

	EAIK::Robot three_parallel(three_parallel_H, three_parallel_P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(three_parallel.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK three parallel - 3,4,5", three_parallel, ee_poses);
}

bool ik_test_5R_345_Parallel_12_Parallel()
{
	Eigen::Matrix<double, 3, 5> three_parallel_H;
	three_parallel_H <<  ez, ez, ex, ex, ex;
	Eigen::Matrix<double, 3, 6> three_parallel_P;
	three_parallel_P <<  ex, ex+ey, ey, ey, ey, ez;

	EAIK::Robot three_parallel(three_parallel_H, three_parallel_P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(three_parallel.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK three parallel - 3,4,5 and 1,2 Parallel", three_parallel, ee_poses);
}

// Axis 2, 3, 4, parallel
bool ik_test_5R_234_Parallel()
{
	Eigen::Matrix<double, 3, 5> three_parallel_H;
	three_parallel_H << ez, ex, ex, ex, ez;
	Eigen::Matrix<double, 3, 6> three_parallel_P;
	three_parallel_P << ez, ey, ey, ey, ey, ey + ex;

	EAIK::Robot three_parallel(three_parallel_H, three_parallel_P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(three_parallel.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK three parallel - 2,3,4", three_parallel, ee_poses);
}


// Two axes intersecting at either end

bool ik_test_5R_12_intersecting()
{
	Eigen::Matrix<double, 3, 5> H;
	H << ey, ex, ey, ez, ey;
	Eigen::Matrix<double, 3, 6> P;
	P << ez, zv, ex+ez, ex+ez, ex+ez, zv;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK axis 1 and 2 intersecting", robot, ee_poses);
}

bool ik_test_5R_45_intersecting()
{
	Eigen::Matrix<double, 3, 5> H;
	H << ey, ez, ey, ex, ey;
	Eigen::Matrix<double, 3, 6> P;
	P << zv, ex + ez, ex + ez, ex + ez, zv, ez;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK axis 4 and 5 intersecting", robot, ee_poses);
}

bool ik_test_5R_12_parallel_45_intersecting()
{
	Eigen::Matrix<double, 3, 5> H;
	H << ex, ex, ey, ex, ey;
	Eigen::Matrix<double, 3, 6> P;
	P <<  ez, ey, ex + ez, ex + ez, zv, ez;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK axis 1 and 2 parallel and axis 4 and 5 intersecting", robot, ee_poses);
}

bool ik_test_5R_12_45_intersecting()
{
	Eigen::Matrix<double, 3, 5> H;
	H << ey, ex, ey, ex, ey;
	Eigen::Matrix<double, 3, 6> P;
	P <<  ez, zv, ex + ez, ex + ez, zv, ez;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK axis 1 and 2 intersecting and axis 4 and 5 intersecting", robot, ee_poses);
}

bool ik_test_5R_23_45_intersecting()
{
	Eigen::Matrix<double, 3, 5> H;
	H << ey, ex, ey, ex, ey;
	Eigen::Matrix<double, 3, 6> P;
	P <<  ez, ex + ez, ex + ey, ex + ez, zv, ez;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK axis 2 and 3 intersecting and axis 4 and 5 intersecting", robot, ee_poses);
}

bool ik_test_5R_23_parallel_45_intersecting()
{
	Eigen::Matrix<double, 3, 5> H;
	H << ex, ey, ey, ex, ey;
	Eigen::Matrix<double, 3, 6> P;
	P <<  ez, ex + ez, ex + ey, ex + ez, zv, ez;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK axis 2 and 3 parallel and axis 4 and 5 intersecting", robot, ee_poses);
}

bool ik_test_5R_23_parallel_45_intersecting_2()
{
	Eigen::Matrix<double, 3, 5> robot_H;
	robot_H << ez, ex, ex, ey, ez;
	Eigen::Matrix<double, 3, 6> robot_P;
	robot_P << ez, ey, ey, ey, ey, ey + ex;

	EAIK::Robot robot(robot_H, robot_P);

	std::vector<IKS::Homogeneous_T> ee_poses;

	ee_poses.reserve(BATCH_SIZE);

	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 4,5 intersecting and 2,3 parallel second test", robot, ee_poses);
}

bool ik_test_5R_spherical_base()
{
	Eigen::Matrix<double, 3, 5> H;
	H << ex, ey, ex, ey, ez;
	Eigen::Matrix<double, 3, 6> P;
	P << zv, ex, zv, ex+ez, ex+ez, zv;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK spherical base", robot, ee_poses);
}

bool ik_test_5R_spherical_base_45_intersecting()
{
	Eigen::Matrix<double, 3, 5> H;
	H << ex, ey, ex, ey, ez;
	Eigen::Matrix<double, 3, 6> P;
	P << zv, ex, zv, ex+ez, ey + ez, ex;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK spherical base and axis 4 and 5 intersecting", robot, ee_poses);
}

bool ik_test_5R_spherical_wrist()
{
	Eigen::Matrix<double, 3, 5> H;
	H << ez, ey, ex, ey, ex;
	Eigen::Matrix<double, 3, 6> P;
	P << zv, ex + ez, ex + ez, zv, ex, zv;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK spherical wrist", robot, ee_poses);
}

bool ik_test_5R_spherical_wrist_12_intersecting()
{
	Eigen::Matrix<double, 3, 5> H;
	H << ez, ey, ex, ey, ex;
	Eigen::Matrix<double, 3, 6> P;
	P << zv, ey + ez, ex + ez, zv, ex, zv;

	EAIK::Robot robot(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK spherical wrist and axis 1 and 2 intersecting", robot, ee_poses);
}

bool ik_test_5R_34_intersecting_23_parallel()
{
	Eigen::Matrix<double, 3, 5> robot_H;
	robot_H << ez, ex, ex, ey, ez;
	Eigen::Matrix<double, 3, 6> robot_P;
	robot_P << ez, ey, ey, ey, ey+ex, ey + ex;

	EAIK::Robot robot(robot_H, robot_P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);

	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 3,4 intersecting and 2,3 parallel", robot, ee_poses);
}

bool ik_test_5R_23_intersecting_34_parallel()
{
	Eigen::Matrix<double, 3, 5> robot_H;
	robot_H << ez, ey, ex, ex, ez;
	Eigen::Matrix<double, 3, 6> robot_P;
	robot_P << ey + ex, ey+ex, ey, ey, ey, ez;

	EAIK::Robot robot(robot_H, robot_P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);

	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 2,3 intersecting and 3,4 parallel", robot, ee_poses);
}

bool ik_test_5R_34_intersecting_23_parallel_45_parallel()
{
	Eigen::Matrix<double, 3, 5> robot_H;
	robot_H << ez, ex, ex, ey, ey;
	Eigen::Matrix<double, 3, 6> robot_P;
	robot_P << ez, ey, ey, ey, ey+ex, ey + ex;

	EAIK::Robot robot(robot_H, robot_P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);

	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(robot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 3,4 intersecting and 2,3 parallel and 4,5 parallel", robot, ee_poses);
}
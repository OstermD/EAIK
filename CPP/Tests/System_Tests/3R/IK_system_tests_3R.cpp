#include <iostream>
#include <chrono>
#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "EAIK.h"
#include "IK_test_utils.h"
#include "IK_system_tests_3R.h"

//
// 3R Tests
//

bool run_3R_Tests()
{
	bool allPass = true;

	// 3R Tests
	allPass &= ik_test_3R_1_2_Parallel();
	allPass &= ik_test_3R_2_3_Parallel();
	allPass &= ik_test_3R_1_2_intersecting_2_3_parallel();
	allPass &= ik_test_3R_1_2_Parallel_2_3_intersecting();
	allPass &= ik_test_3R_generic();
	allPass &= ik_test_3R_1_2_intersecting();
	allPass &= ik_test_3R_2_3_intersecting();
	allPass &= ik_test_3R_1_2_intersecting_2_3_intersecting();
	allPass &= ik_test_3R_1_2_3_intersecting();
	allPass &= ik_test_3R_1_2_3_intersecting_2();
	allPass &= ik_test_3R_1_2_3_parallel();
	allPass &= ik_test_3R_1_2_3_parallel_2();

	std::cout<< std::endl<<"====================== RESULT: ======================"<<std::endl;

	if(allPass)
	{
		std::cout<< "                     3R  PASSING                     "<<std::endl;
	}
	else
	{
		std::cout<< "                     3R   FAILED                     "<<std::endl;
	}

	std::cout<< "====================================================="<<std::endl;

	return allPass;
}

bool ik_test_3R_PUMA_locked_wrist()
{
	// Robot configuration for spherical wrist with second and third axis intersecting
	Eigen::Matrix<double, 3, 6> H;
	H << ez, -ey, -ey, ez, -ey, ez;
	Eigen::Matrix<double, 3, 7> P;
	P << 0.54864 * ez, -0.14224 * ey + 0.07493 * ez, 0.4318 * ex - 0.0254 * ey, 0.0381 * ey + 0.3517 * ez, 0.080299 * ez, 0.05334 * ez, zv;

	// Lock puma wrist such that it behaves like a 3R manipulator
	double q4 = rand_angle();
	double q5 = rand_angle();
	double q6 = rand_angle();
	EAIK::Robot puma_locked_wrist(H, P, Eigen::Matrix<double, 3, 3>::Identity(), {std::pair<int, double>(4, q5), std::pair<int, double>(3, q4), std::pair<int, double>(5, q6)});
	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(puma_locked_wrist.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), q4, q5, q6}));
	}

	return evaluate_test<EAIK::Robot>("IK 3R puma - locked wrist", puma_locked_wrist, ee_poses);
}

bool ik_test_3R_1_2_Parallel()
{
	Eigen::Matrix<double, 3, 3> H;
	H << ez, ez, ex;
	Eigen::Matrix<double, 3, 4> P;
	P << ez, ex, ey, ey;

	EAIK::Robot two_parallel(H, P);
	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(two_parallel.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 3R - 1,2 Parallel", two_parallel, ee_poses);
}

bool ik_test_3R_2_3_Parallel()
{
	Eigen::Matrix<double, 3, 3> H;
	H << ez, ex, ex;
	Eigen::Matrix<double, 3, 4> P;
	P << ex, ez, ez, ez;

	EAIK::Robot two_parallel(H, P);
	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(two_parallel.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle()}));

	}

	return evaluate_test<EAIK::Robot>("IK 3R - 2,3 Parallel", two_parallel, ee_poses);
}

bool ik_test_3R_1_2_intersecting_2_3_parallel()
{
	// First and second axis intersecting; Second and third axis Parallell
	Eigen::Matrix<double, 3, 3> H;
	H << ez, ey, ey;
	Eigen::Matrix<double, 3, 4> P;
	P << 0.32 * ex + 0.78 * ez, 1.075 * ez, 1.1425 * ex + 0.2 * ez, zv;

	EAIK::Robot _1_2_intersecting_2_3_parallel(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(_1_2_intersecting_2_3_parallel.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 3R - 1,2 intersecting 2,3 parallel", _1_2_intersecting_2_3_parallel, ee_poses);
}

bool ik_test_3R_1_2_Parallel_2_3_intersecting()
{
	Eigen::Matrix<double, 3, 3> H;
	H << ez, ez, ey;
	Eigen::Matrix<double, 3, 4> P;
	P << ez, ex, ey, ey;

	EAIK::Robot _1_2_parallel_2_3_intersecting(H, P);
	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(_1_2_parallel_2_3_intersecting.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle()}));

	}

	return evaluate_test<EAIK::Robot>("IK 3R - 1,2 parallel 2,3 intersecting", _1_2_parallel_2_3_intersecting, ee_poses);
}

bool ik_test_3R_1_2_3_intersecting()
{
	// Spherical wrist case with first and last axes parallel
	Eigen::Matrix<double, 3, 3> H;
	H << ex, ey, ex;
	Eigen::Matrix<double, 3, 4> P;
	P << zv, zv, 0.2 * ex, zv;

	EAIK::Robot _1_2_3_intersecting(H, P);
	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(_1_2_3_intersecting.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle()}));

	}

	return evaluate_test<EAIK::Robot>("IK 3R - 1,2,3 Intersecting - Spherical Wrist (2 Parallel)", _1_2_3_intersecting, ee_poses);
}

bool ik_test_3R_1_2_3_intersecting_2()
{
	// Spherical wrist case with three orthonormal axes
	Eigen::Matrix<double, 3, 3> H;
	H << ex, ey, ez;
	Eigen::Matrix<double, 3, 4> P;
	P << zv, zv, ez, 0.2 * ex;

	EAIK::Robot _1_2_3_intersecting(H, P);
	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(_1_2_3_intersecting.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle()}));

	}

	return evaluate_test<EAIK::Robot>("IK 3R - 1,2,3 Intersecting - Spherical Wrist (3 Orthonormal)", _1_2_3_intersecting, ee_poses);
}


bool ik_test_3R_1_2_intersecting_2_3_intersecting()
{
	// No spherical wrist, but 1 intersecting 2 and 2 intersecting 3
	Eigen::Matrix<double, 3, 3> H;
	H << ez, ey, ex;
	Eigen::Matrix<double, 3, 4> P;
	P << zv, zv, 0.2 * ey, zv;

	EAIK::Robot _1_2_intersecting_2_3_intersecting(H, P);
	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(_1_2_intersecting_2_3_intersecting.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle()}));

	}

	return evaluate_test<EAIK::Robot>("IK 3R - 1,2 Intersecting, 2,3 Intersecting", _1_2_intersecting_2_3_intersecting, ee_poses);
}

bool ik_test_3R_1_2_3_parallel()
{
	// Three parallel axes (Inherently redundant robot!)
	Eigen::Matrix<double, 3, 3> H;
	H << ez, ez, ez;
	Eigen::Matrix<double, 3, 4> P;
	P << zv, ez, ez, zv;

	EAIK::Robot _1_2_3_parallel(H, P);
	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(_1_2_3_parallel.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 3R - 1,2,3 Parallel", _1_2_3_parallel, ee_poses);
}

bool ik_test_3R_1_2_3_parallel_2()
{
	// Three parallel axes with offsets
	Eigen::Matrix<double, 3, 3> H;
	H << ez, ez, ez;
	Eigen::Matrix<double, 3, 4> P;
	P << zv, ez+ex, ez+ey, zv;

	EAIK::Robot _1_2_3_parallel(H, P);
	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(_1_2_3_parallel.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 3R - 1,2,3 Parallel + Offsets", _1_2_3_parallel, ee_poses);
}

bool ik_test_3R_1_2_intersecting()
{
	// First and second Second axis intersecting
	Eigen::Matrix<double, 3, 3> H;
	H << ez, ey, ex;
	Eigen::Matrix<double, 3, 4> P;
	P << 0.32 * ex + 0.78 * ez, 1.075 * ez, 1.1425 * ey + 0.2 * ez, zv;

	EAIK::Robot _1_2_intersecting(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(_1_2_intersecting.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 3R - 1,2 intersecting", _1_2_intersecting, ee_poses);
}

bool ik_test_3R_2_3_intersecting()
{
	// Second and third axis intersecting
	Eigen::Matrix<double, 3, 3> H;
	H << ez, -ey, -ex;
	Eigen::Matrix<double, 3, 4> P;
	P << 0.54864 * ez, -0.14224 * ex, 0.07493 * ey, 0.4318 * ex - 0.0254 * ey;
	EAIK::Robot two_three_intersecting(H, P);
	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(two_three_intersecting.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 3R - 2,3 intersecting", two_three_intersecting, ee_poses);
}

bool ik_test_3R_generic()
{
	// No (consecutive) Parallel, No intersecting Axes
	Eigen::Matrix<double, 3, 3> H;
	H << ey, ez, ey;
	Eigen::Matrix<double, 3, 4> P;
	P << zv, ex + ez, ex + ez, ex + ez;

	EAIK::Robot generic(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(generic.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test<EAIK::Robot>("IK 3R - No Parallel; No Intersecting", generic, ee_poses);
}
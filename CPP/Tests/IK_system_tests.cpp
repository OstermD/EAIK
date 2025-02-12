#include <iostream>
#include <chrono>
#include <string>
#include <eigen3/Eigen/Dense>
#include <random>
#include <vector>

#include "EAIK.h"
#include "kinematic_remodeling.h"

#define ERROR_PASS_EPSILON 1e-5
#define BATCH_SIZE 100

const Eigen::Vector3d zv(0, 0, 0);
const Eigen::Vector3d ex(1, 0, 0);
const Eigen::Vector3d ey(0, 1, 0);
const Eigen::Vector3d ez(0, 0, 1);

// NR Tests
bool ik_test_7R_KUKA_R800();
bool ik_test_7R_Panda();

// 6R Tests
bool ik_test_puma();
bool ik_test_PUMA_DH();
bool ik_test_IRB6640();
bool ik_test_spherical();
bool ik_test_puma_reversed();
bool ik_test_spherical_reversed();
bool ik_test_IRB6640_reversed();

bool ik_test_234_Parallel();
bool ik_test_123_Parallel();
bool ik_test_123_Parallel_56_Intersecting();
bool ik_test_UR5();
bool ik_test_345_Parallel();
bool ik_test_456_Parallel();
bool ik_test_456_Parallel_12_Intersecting();
bool ik_test_345_Parallel();

bool ik_test_two_Sphericals();

// 3R Tests
bool ik_test_3R_generic();
bool ik_test_3R_1_2_Parallel();
bool ik_test_3R_2_3_Parallel();
bool ik_test_3R_1_2_intersecting_2_3_parallel();
bool ik_test_3R_1_2_Parallel_2_3_intersecting();
bool ik_test_3R_1_2_intersecting_2_3_intersecting();
bool ik_test_3R_1_2_3_intersecting();
bool ik_test_3R_1_2_3_intersecting_2();
bool ik_test_3R_1_2_3_parallel();
bool ik_test_3R_1_2_3_parallel_2();
bool ik_test_3R_1_2_intersecting();
bool ik_test_3R_2_3_intersecting();
bool ik_test_3R_PUMA_locked_wrist();

// I/O Component tests
bool test_eigen_IO();
bool test_batched_IK();

// Utility-Functions

bool evaluate_test(const std::string &name_test, const EAIK::Robot &robot, const std::vector<IKS::Homogeneous_T> &ee_poses);

// Create a random angle
double rand_angle()
{
	std::random_device rd;
	std::default_random_engine eng(rd());
	std::uniform_real_distribution<double> distr(0, 1);

	double theta = distr(eng) * 2 * M_PI - M_PI;

	return theta;
}

int main(int argc, char *argv[])
{
	bool allPass = true;
	allPass &= ik_test_234_Parallel();

	allPass &= ik_test_7R_Panda();
	allPass &= ik_test_7R_KUKA_R800();
	allPass &= ik_test_3R_PUMA_locked_wrist();
	
	// 6R Tests
	allPass &= ik_test_puma();
	allPass &= ik_test_PUMA_DH();
	allPass &= ik_test_IRB6640();
	allPass &= ik_test_spherical();
	allPass &= ik_test_puma_reversed();
	allPass &= ik_test_spherical_reversed();
	allPass &= ik_test_IRB6640_reversed();

	allPass &= ik_test_123_Parallel_56_Intersecting();
	allPass &= ik_test_UR5();
	allPass &= ik_test_two_Sphericals();

	allPass &= ik_test_345_Parallel();
	allPass &= ik_test_456_Parallel_12_Intersecting();

	//// 3R Tests
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

	// I/O Tests
	allPass &= test_eigen_IO();
	allPass &= test_batched_IK();

	std::cout<< std::endl<<"====================== RESULT: ======================"<<std::endl;

	if(allPass)
	{
		std::cout<< "                     ALL PASSING                     "<<std::endl;
	}
	else
	{
		std::cout<< "                     SOME FAILED                     "<<std::endl;
	}

	std::cout<< "==================================================="<<std::endl;

	return 0;
}


//
// I/O Component tests
//


bool test_batched_IK()
{
	// Robot configuration for spherical wrist with second and third axis intersecting
	Eigen::Matrix<double, 3, 6> H;
	H << ez, -ey, -ey, ez, -ey, ez;
	Eigen::Matrix<double, 3, 7> P;
	P << 0.54864 * ez, -0.14224 * ey + 0.07493 * ez, 0.4318 * ex - 0.0254 * ey, 0.0381 * ey + 0.3517 * ez, 0.080299 * ez, 0.05334 * ez, zv;

	EAIK::Robot spherical(H, P);
	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	std::vector<IKS::IK_Solution> solutions_reference;
	solutions_reference.reserve(BATCH_SIZE);

	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(spherical.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
		solutions_reference.push_back(spherical.calculate_IK(ee_poses.back()));
	}
	const std::vector<IKS::IK_Solution>& solutions_batched = spherical.calculate_IK_batched(ee_poses, 6);

	// Make sure the solutions match up
	bool is_equal = true;
	for(unsigned i = 0; i < solutions_reference.size(); i++)
	{
		const std::vector<std::vector<double>>& Q_batched = solutions_batched.at(i).Q;
		const std::vector<std::vector<double>>& Q_reference = solutions_reference.at(i).Q;

		for(unsigned j = 0; j < Q_reference.size(); j++)
		{
			is_equal &= (Q_batched.at(j) == Q_reference.at(j)); 
			is_equal &= solutions_batched.at(i).is_LS_vec.at(j) == solutions_reference.at(i).is_LS_vec.at(j);
		}
	}

	std::cout << "\n===== Test [Batched-IK]: ";
	if(is_equal)
	{
		std::cout << "[PASS] =====" << std::endl;
	}else
	{
		std::cout << "[FAIL] =====" << std::endl;
	}

	return is_equal;
}

// Test Eigen Representation of IK-Solution object
bool test_eigen_IO()
{
	// Puma config
	Eigen::Matrix<double, 3, 6> puma_H;
	puma_H << ez, -ey, -ey, ez, ey, ez;
	Eigen::Matrix<double, 3, 7> puma_P;
	puma_P << 0.62357 * ez, zv, 0.4318 * ex - 0.16764 * ey, 0.432089 * ez + 0.0381 * ey, zv, zv, 0.05334 * ez;

	EAIK::Robot puma(puma_H, puma_P);
	std::vector<double> joint_angles{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()};
	IKS::Homogeneous_T ee_pose_vector = puma.fwdkin(joint_angles);

	IKS::Homogeneous_T ee_pose_eigen;
	Eigen::VectorXd eigen_joint_angles(6);
	eigen_joint_angles << joint_angles.at(0), joint_angles.at(1), joint_angles.at(2), joint_angles.at(3), joint_angles.at(4), joint_angles.at(5);
	ee_pose_eigen = puma.fwdkin_Eigen(eigen_joint_angles);

	bool is_test_passed = true;

	std::cout << "\n===== Test [EIGEN-IO]: ";

	if((ee_pose_eigen-ee_pose_vector).norm() > 1e-15)
	{
		std::cout<<"Error: Forward kinematics for Eigen and Vector representation are not invariant! ";
		is_test_passed = false;
	}
	else
	{
		IKS::IK_Eigen_Solution eigen_sol = puma.calculate_Eigen_IK(ee_pose_eigen);
		IKS::IK_Solution vector_sol = puma.calculate_IK(ee_pose_vector);

		if(eigen_sol.is_LS_vec.rows() != eigen_sol.Q.rows())
		{
			std::cout<<"Error: is_LS_vec's rows don't correspond to Q's rows! ";
			is_test_passed = false;
		}
		else if(eigen_sol.Q.rows() != vector_sol.Q.size())
		{
			std::cout<<"Error: Eigen solution set doesn't correspond to Vector solution set! ";
			is_test_passed = false;
		}
		else
		{
			for(unsigned i = 0; i < vector_sol.Q.size(); ++i)
			{
				if(eigen_sol.is_LS_vec(i) != vector_sol.is_LS_vec.at(i))
				{
					std::cout<<"Error: Eigen LS markings don't correspond to Vector LS markings! ";
					is_test_passed = false;
					break;
				}
				else if((puma.fwdkin_Eigen(eigen_sol.Q.row(i)) - puma.fwdkin(vector_sol.Q.at(i))).norm() > 1e-15)
				{
					std::cout<<"Error: Eigen IK solution doesn't correspond to Vector solution! ";
					is_test_passed = false;
					break;
				}
			}
		}
	}

	if(is_test_passed)
	{
		std::cout << "[PASS] =====" << std::endl;
	}else
	{
		std::cout << "[FAIL] =====" << std::endl;
	}

	return is_test_passed;
}


//
// NR Tests (by Joint locking)
//

bool ik_test_7R_Panda()
{
	// Robot configuration for Franka Panda
	Eigen::Matrix<double, 3, 7> H;
	H << ez, ey, ez, -ey, ez, -ey, -ez;
	Eigen::Matrix<double, 3, 8> P;
	P << 0.333 * ez, zv, 0.316 * ez, 0.0825*ex, - 0.0825*ex + 0.384 * ez, zv, 0.088*ex, zv;

	// Panda with q4 locked in random configuration
	double q4_angle = 0;//rand_angle();
	EAIK::Robot panda(H, P, Eigen::Matrix<double, 3, 3>::Identity(), {std::pair<int, double>(3, q4_angle)});

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(panda.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), q4_angle, rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK 7R spherical wrist - Panda", panda, ee_poses);
}


bool ik_test_7R_KUKA_R800()
{
	// Robot configuration for KUKA LBR iiwa 7 R800
	Eigen::Matrix<double, 3, 7> H;
	H << ez, ey, ez, -ey, ez, ey, ez;
	Eigen::Matrix<double, 3, 8> P;
	P << (0.15 + 0.19) * ez, zv, 0.21 * ez, 0.19 * ez, (0.21 + 0.19) * ez, zv, zv, (0.081 + 0.045) * ez;

	// Kuka R800 with q3 locked in random configuration
	double q3_angle = rand_angle();
	EAIK::Robot kukaR800(H, P, Eigen::Matrix<double, 3, 3>::Identity(), {std::pair<int, double>(2, q3_angle)});

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(kukaR800.fwdkin(std::vector{rand_angle(), rand_angle(), q3_angle, rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK 7R spherical wrist - KUKA LBR iiwa 7 R800", kukaR800, ee_poses);
}


//
// 3R Tests
//


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

	return evaluate_test("IK 3R puma - locked wrist", puma_locked_wrist, ee_poses);
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

	return evaluate_test("IK 3R - 1,2 Parallel", two_parallel, ee_poses);
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

	return evaluate_test("IK 3R - 2,3 Parallel", two_parallel, ee_poses);
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

	return evaluate_test("IK 3R - 1,2 intersecting 2,3 parallel", _1_2_intersecting_2_3_parallel, ee_poses);
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

	return evaluate_test("IK 3R - 1,2 parallel 2,3 intersecting", _1_2_parallel_2_3_intersecting, ee_poses);
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

	return evaluate_test("IK 3R - 1,2,3 Intersecting - Spherical Wrist (2 Parallel)", _1_2_3_intersecting, ee_poses);
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

	return evaluate_test("IK 3R - 1,2,3 Intersecting - Spherical Wrist (3 Orthonormal)", _1_2_3_intersecting, ee_poses);
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

	return evaluate_test("IK 3R - 1,2 Intersecting, 2,3 Intersecting", _1_2_intersecting_2_3_intersecting, ee_poses);
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

	return evaluate_test("IK 3R - 1,2,3 Parallel", _1_2_3_parallel, ee_poses);
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

	return evaluate_test("IK 3R - 1,2,3 Parallel + Offsets", _1_2_3_parallel, ee_poses);
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

	return evaluate_test("IK 3R - 1,2 intersecting", _1_2_intersecting, ee_poses);
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

	return evaluate_test("IK 3R - 2,3 intersecting", two_three_intersecting, ee_poses);
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

	return evaluate_test("IK 3R - No Parallel; No Intersecting", generic, ee_poses);
}

//
// 6R Tests
//


// Axis 1, 2, 3, parallel
bool ik_test_123_Parallel()
{
	const Eigen::Vector3d zv(0, 0, 0);
	const Eigen::Vector3d ex(1, 0, 0);
	const Eigen::Vector3d ey(0, 1, 0);
	const Eigen::Vector3d ez(0, 0, 1);

	Eigen::Matrix<double, 3, 6> bot_H;
	bot_H << ey, ey, ey, ez, ex, ez;
	Eigen::Matrix<double, 3, 7> bot_P;
	bot_P << zv, ex, ez-ex, -ex, ez+ey, ex+ey, zv;
	
	EAIK::Robot three_parallel(bot_H, bot_P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(three_parallel.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));

	}

	return evaluate_test("IK three parallel - 1,2,3", three_parallel, ee_poses);
}

// Axis 1, 2, 3, parallel, 5,6 Intersecting
bool ik_test_123_Parallel_56_Intersecting()
{
	const Eigen::Vector3d zv(0, 0, 0);
	const Eigen::Vector3d ex(1, 0, 0);
	const Eigen::Vector3d ey(0, 1, 0);
	const Eigen::Vector3d ez(0, 0, 1);

	Eigen::Matrix<double, 3, 6> bot_H;
	bot_H << ey, ey, ey, ez, ex, ez;
	Eigen::Matrix<double, 3, 7> bot_P;
	bot_P << zv, ex, ez-ex, -ex, ez+ey, ex, zv;
	
	EAIK::Robot three_parallel(bot_H, bot_P);
	std::vector<IKS::Homogeneous_T> ee_poses;

	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(three_parallel.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}
	
	return evaluate_test("IK three parallel - 1,2,3 Parallel - 5,6 Intersecting", three_parallel, ee_poses);
}

// Axis 2, 3, 4, parallel
bool ik_test_UR5()
{
	const Eigen::Vector3d zv(0, 0, 0);
	const Eigen::Vector3d ex(1, 0, 0);
	const Eigen::Vector3d ey(0, 1, 0);
	const Eigen::Vector3d ez(0, 0, 1);

	Eigen::Matrix<double, 3, 6> ur5_H;
	ur5_H << ez, ey, ey, ey, -ez, ey;
	Eigen::Matrix<double, 3, 7> ur5_P;
	ur5_P << 0.089159*ez, 0.13585*ey, 0.425*ex-0.1197*ey, 0.39225*ex,0.093*ey, -0.09465*ez, zv;

	EAIK::Robot three_parallel(ur5_H, ur5_P);

	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(three_parallel.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));

	}

	return evaluate_test("IK UR5 three parallel - 2,3,4", three_parallel, ee_poses);
}

// Axis 2, 3, 4, parallel
bool ik_test_234_Parallel()
{
	Eigen::Matrix<double, 3, 6> three_parallel_H;
	three_parallel_H << ez, ex, ex, ex, ez, ex;
	Eigen::Matrix<double, 3, 7> three_parallel_P;
	three_parallel_P << ez, ey, ey, ey, ey, ey + ex, ex;

	EAIK::Robot three_parallel(three_parallel_H, three_parallel_P);

	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(three_parallel.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));

	}

	return evaluate_test("IK three parallel - 2,3,4", three_parallel, ee_poses);
}

// Axis 4, 5, 6, parallel
bool ik_test_456_Parallel()
{
	const Eigen::Vector3d zv(0, 0, 0);
	const Eigen::Vector3d ex(1, 0, 0);
	const Eigen::Vector3d ey(0, 1, 0);
	const Eigen::Vector3d ez(0, 0, 1);

	Eigen::Matrix<double, 3, 6> bot_H;
	bot_H << ez, ex, ez, ey, ey, ey;
	Eigen::Matrix<double, 3, 7> bot_P;
	bot_P << zv, ex+ey, ez+ey, -ex, ez-ex, ex, zv;
	
	EAIK::Robot three_parallel(bot_H, bot_P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(three_parallel.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK three parallel - 4,5,6", three_parallel, ee_poses);
}

// Axis 4, 5, 6, parallel, 1,2 Intersecting
bool ik_test_456_Parallel_12_Intersecting()
{
	const Eigen::Vector3d zv(0, 0, 0);
	const Eigen::Vector3d ex(1, 0, 0);
	const Eigen::Vector3d ey(0, 1, 0);
	const Eigen::Vector3d ez(0, 0, 1);

	Eigen::Matrix<double, 3, 6> bot_H;
	bot_H << ez, ex, ez, ey, ey, ey; 
	Eigen::Matrix<double, 3, 7> bot_P;
	bot_P << zv, ex, ez+ey, -ex, ez-ex, ex, zv;
	
	EAIK::Robot three_parallel(bot_H, bot_P);
	std::vector<IKS::Homogeneous_T> ee_poses;

	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(three_parallel.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}
	
	return evaluate_test("IK three parallel - 4,5,6 Parallel - 1,2 Intersecting", three_parallel, ee_poses);
}

// Axis 3, 4, 5, parallel
bool ik_test_345_Parallel()
{
	Eigen::Matrix<double, 3, 6> three_parallel_H;
	three_parallel_H <<  ex, ez, ex, ex, ex, ez;
	Eigen::Matrix<double, 3, 7> three_parallel_P;
	three_parallel_P <<  ex, ex+ey, ey, ey, ey, ey, ez;

	EAIK::Robot three_parallel(three_parallel_H, three_parallel_P);

	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(three_parallel.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));

	}

	return evaluate_test("IK three parallel - 3,4,5", three_parallel, ee_poses);
}

// spherical wrist at the bottom, intersecting axes 5,6, parallel 4,5, intersecting 3,4
bool ik_test_IRB6640_reversed()
{
	// Robot configuration for spherical wris
	Eigen::Matrix<double, 3, 6> IRB6640_H;
	IRB6640_H << ex, ey, ex, ey, ey, ez;
	Eigen::Matrix<double, 3, 7> IRB6640_P;
	IRB6640_P << zv, 0.2*ex, zv, zv,  1.1425 * ex + 0.2 * ez,1.075 * ez,0.32 * ex + 0.78 * ez;

	EAIK::Robot IRB6640(IRB6640_H, IRB6640_P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(IRB6640.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK spherical wrist - IRB6640 reversed", IRB6640, ee_poses);
}

// Spherical wrist at the bottom - no other intersecting/parallel consecutive axes
bool ik_test_spherical_reversed()
{
	// Robot configuration for spherical wrist with
	Eigen::Matrix<double, 3, 6> H;
	H << ex, ey, ex, ey, ez, ey;
	Eigen::Matrix<double, 3, 7> P;
	P << zv, ex, zv, ex+ez, ex+ez, ex+ez, zv;

	EAIK::Robot spherical(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(spherical.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK spherical wrist - spherical reversed", spherical, ee_poses);
}

// Puma kinematic but with the wrist at the bottom
bool ik_test_puma_reversed()
{
	// Robot configuration for spherical wrist with second and third axis intersecting
	Eigen::Matrix<double, 3, 6> H;
	H << ez, -ey, ez, -ey, -ey, ez;
	Eigen::Matrix<double, 3, 7> P;
	P << zv, 0.05334 * ez, 0.080299 * ez, 0.0381 * ey + 0.3517 * ez, 0.4318 * ex - 0.0254 * ey, -0.14224 * ey + 0.07493 * ez,0.54864 * ez;

	EAIK::Robot spherical(H, P);
	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(spherical.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK spherical wrist - puma reversed", spherical, ee_poses);
}

// spherical wrist, intersecting axes 1,2, parallel 2,3, intersecting 3,4
bool ik_test_IRB6640()
{
	// Robot configuration for spherical wris
	Eigen::Matrix<double, 3, 6> IRB6640_H;
	IRB6640_H << ez, ey, ey, ex, ey, ex;
	Eigen::Matrix<double, 3, 7> IRB6640_P;
	IRB6640_P << 0.32 * ex + 0.78 * ez, 1.075 * ez, 1.1425 * ex + 0.2 * ez, zv, zv, 0.2 * ex, zv;

	EAIK::Robot IRB6640(IRB6640_H, IRB6640_P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(IRB6640.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK spherical wrist - IRB6640", IRB6640, ee_poses);
}

bool ik_test_spherical()
{
	// Robot configuration for spherical wrist with
	Eigen::Matrix<double, 3, 6> H;
	H << ey, ez, ey, ex, ey, ex;
	Eigen::Matrix<double, 3, 7> P;
	P << zv, ex + ez, ex + ez, ex + ez, zv, ex, zv;

	EAIK::Robot spherical(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(spherical.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK spherical wrist - spherical", spherical, ee_poses);
}

bool ik_test_puma()
{
	// Robot configuration for spherical wrist with second and third axis intersecting
	Eigen::Matrix<double, 3, 6> H;
	H << ez, -ey, -ey, ez, -ey, ez;
	Eigen::Matrix<double, 3, 7> P;
	P << 0.54864 * ez, -0.14224 * ey + 0.07493 * ez, 0.4318 * ex - 0.0254 * ey, 0.0381 * ey + 0.3517 * ez, 0.080299 * ez, 0.05334 * ez, zv;

	EAIK::Robot spherical(H, P);
	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(spherical.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK spherical wrist - puma", spherical, ee_poses);
}

bool ik_test_PUMA_DH()
{
	// Robot configuration for PUMA 560 according to https://de.mathworks.com/help/robotics/ug/build-manipulator-robot-using-kinematic-dh-parameters.html
	Eigen::VectorXd a(6);
	a << 0, 0.4318, 0.0203, 0, 0, 0;
	Eigen::VectorXd alpha(6);
	alpha << M_PI/2, 0, -M_PI/2, M_PI/2, -M_PI/2, 0;
	Eigen::VectorXd d(6);
	d<< 0,0,0.15005, 0.4318, 0, 0;

	EAIK::Robot puma560(alpha, a, d);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(puma560.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK 6R spherical wrist - Puma DH parametrized", puma560, ee_poses);
}

// Two spherical wrists
bool ik_test_two_Sphericals()
{
	const Eigen::Vector3d zv(0, 0, 0);
	const Eigen::Vector3d ex(1, 0, 0);
	const Eigen::Vector3d ey(0, 1, 0);
	const Eigen::Vector3d ez(0, 0, 1);

	Eigen::Matrix<double, 3, 6> H;
	H << ey, -ez, -ey, -ey, -ez, ey;
	Eigen::Matrix<double, 3, 7> P;
	P << 0.0528192*ex, zv, zv, 0.140253*ex, zv, zv, zv;

	EAIK::Robot bot(H, P);

	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(bot.fwdkin(std::vector{rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));

	}

	return evaluate_test("IK two spherical wrists", bot, ee_poses);
}

bool evaluate_test(const std::string &name_test, const EAIK::Robot &robot, const std::vector<IKS::Homogeneous_T> &ee_poses)
{
	IKS::IK_Solution solution;
	const auto start = std::chrono::steady_clock::now();

	// Do once for timing test
	for (const auto &pose : ee_poses)
	{
		solution = robot.calculate_IK(pose);
	}

	const auto end = std::chrono::steady_clock::now();
	unsigned long time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

	// Do once for precision test
	double sum_error = 0;
	double max_error = 0;
	unsigned total_non_LS_solutions = 0;
	unsigned total_LS_solutions = 0;
	unsigned total_no_solution = 0;
	for (const auto &pose : ee_poses)
	{
		solution = robot.calculate_IK(pose);
		unsigned non_LS_solutions = 0;

		for (unsigned i = 0; i < solution.Q.size(); i++)
		{
			for(const auto& q : solution.Q.at(i))
			{
				if(std::isnan(q))
				{
					throw std::runtime_error("Solution contained NAN!");
				}
			}

			// Only account for non-LS-solutions in this test
			if (!solution.is_LS_vec.at(i))
			{
				IKS::Homogeneous_T result = robot.fwdkin(solution.Q.at(i));

				double error = (result - pose).norm();
				max_error = max_error < error ? error : max_error;
				sum_error += error;
				non_LS_solutions++;
				total_non_LS_solutions++;
			}
		}

		if (non_LS_solutions == 0)
		{
			std::vector<double> smallest_error_LS;
			double smallest_error = std::numeric_limits<double>::infinity();
			for (unsigned i = 0; i < solution.Q.size(); i++)
			{
				IKS::Homogeneous_T result = robot.fwdkin(solution.Q.at(i));

				double error = (result - pose).norm();
				// Only account for non-LS-solutions in this test
				if (error < smallest_error)
				{
					smallest_error = error;
					smallest_error_LS = solution.Q.at(i);
				}
			}
			if(solution.Q.size() > 0)
			{
				max_error = max_error < smallest_error ? smallest_error : max_error;
				
				sum_error += smallest_error;
				total_LS_solutions++;
			}
			else
			{
				total_no_solution++;
			}
		}
	}

	double avg_error = std::numeric_limits<double>::infinity();
	
	if(total_LS_solutions+total_non_LS_solutions > 0)
	{
		avg_error = sum_error / (total_non_LS_solutions+total_LS_solutions);
	}
	const bool is_passed{std::fabs(max_error) < ERROR_PASS_EPSILON &&
						 std::fabs(avg_error) < ERROR_PASS_EPSILON};
	std::cout << "\n===== Test [" << name_test << "]: ";
	if (is_passed)
	{
		std::cout << "[PASS] =====" << std::endl;
	}
	else
	{
		std::cout << "[FAIL] =====" << std::endl;
	}
	std::cout << "\tAverage error: " << avg_error << std::endl;
	std::cout << "\tMaximum error: " << max_error << std::endl;
	std::cout << "\tNum LS solutions:  " << total_LS_solutions << std::endl;
	std::cout << "\tNum NO solutions:  " << total_no_solution << std::endl;
	std::cout << "===== Average solution time (nanoseconds): " << time / BATCH_SIZE << " =====" << std::endl;

	return is_passed;
}
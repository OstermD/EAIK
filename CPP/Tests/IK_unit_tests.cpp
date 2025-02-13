#include <iostream>
#include <chrono>
#include <string>
#include <eigen3/Eigen/Dense>
#include <random>
#include <vector>

#include "IKS.h"
#include "kinematic_utils.h"
#include "kinematic_remodeling.h"

#define ERROR_PASS_EPSILON 1e-6
#define BATCH_SIZE 100

const Eigen::Vector3d zv(0, 0, 0);
const Eigen::Vector3d ex(1, 0, 0);
const Eigen::Vector3d ey(0, 1, 0);
const Eigen::Vector3d ez(0, 0, 1);

// Spherical wrist
bool ik_test_SPHERICAL_1_2_P();
bool ik_test_SPHERICAL_2_3_P();
bool ik_test_SPHERICAL_1_3_P();
bool ik_test_SPHERICAL();
bool ik_test_SPHERICAL_1_2_I();
bool ik_test_SPHERICAL_2_3_I();
bool ik_test_PUMA();
bool ik_test_SPHERICAL_2_3_P_REDUNDANT();

bool test_inv_kin_chain();
bool test_partial_joint_parametrization();

// No spherical wrist
bool ik_test_3P();

bool evaluate_test(const std::string &name_test,
				   const IKS::General_Robot &robot,
				   const std::vector<IKS::Homogeneous_T> &ee_poses);

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
	test_inv_kin_chain();
	test_partial_joint_parametrization();
	
	// IK tests
	ik_test_SPHERICAL_1_2_P();
	ik_test_SPHERICAL_2_3_P();
	ik_test_SPHERICAL_1_3_P();
	ik_test_SPHERICAL();
	ik_test_SPHERICAL_1_2_I();
	ik_test_SPHERICAL_2_3_I();
	ik_test_PUMA();
	ik_test_3P();

	// Redundancy test might create warning output on std::out
	ik_test_SPHERICAL_2_3_P_REDUNDANT();
	
	return 0;
}

// Test partial parametrization of the kinematic chain
bool test_partial_joint_parametrization()
{
	// Puma config
	Eigen::Matrix<double, 3, 6> puma_H;
	puma_H << ez, -ey, -ey, ez, ey, ez;
	Eigen::Matrix<double, 3, 7> puma_P;
	puma_P << 0.62357 * ez, zv, 0.4318 * ex - 0.16764 * ey, 0.432089 * ez + 0.0381 * ey, zv, zv, 0.05334 * ez;

	IKS::General_Robot puma(puma_H, puma_P);

	std::vector<double> joint_angles {rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()};

	IKS::Homogeneous_T reference_ee_pose = puma.fwdkin(joint_angles);
	
	bool is_passed = true;
	std::vector<std::pair<int,double>> partial_parametrization;
	for(unsigned i = 0; i < 6; i++)
	{
		partial_parametrization.push_back({i, joint_angles.at(i)});
		const auto&[H_partial, P_partial, R6T] = EAIK::partial_joint_parametrization(puma_H, puma_P, partial_parametrization);

		IKS::General_Robot partial_puma(H_partial, P_partial);
		IKS::Homogeneous_T partial_ee_pose = partial_puma.fwdkin(std::vector<double>(joint_angles.begin()+i+1,joint_angles.end()));
		partial_ee_pose.block<3,3>(0,0) *= R6T;

		is_passed &= (partial_ee_pose-reference_ee_pose).isZero(1e-14);
	}

	std::cout << "\n===== Test [Partial Kinematic Parametrization]: ";
	if (is_passed)
	{
		std::cout << "[PASS] =====" << std::endl;
	}
	else
	{
		std::cout << "[FAIL] =====" << std::endl;
	}
	return is_passed;
}

// Test kinematic chain inversion
bool test_inv_kin_chain()
{
	// Puma config
	Eigen::Matrix<double, 3, 6> puma_H;
	puma_H << ez, -ey, -ey, ez, ey, ez;
	Eigen::Matrix<double, 3, 7> puma_P;
	puma_P << 0.62357 * ez, zv, 0.4318 * ex - 0.16764 * ey, 0.432089 * ez + 0.0381 * ey, zv, zv, 0.05334 * ez;

	IKS::General_6R puma(puma_H, puma_P);

	// Build kinematically reversed robot
	const auto&[H_reversed, P_reversed] = IKS::reverse_kinematic_chain(puma_H, puma_P);
	IKS::General_6R puma_reversed(H_reversed, P_reversed);

	bool is_Pass = true;
	for(unsigned i = 0; i < BATCH_SIZE; ++i)
	{
		std::vector<double> joint_angles{rand_angle(), rand_angle(),rand_angle(),rand_angle(),rand_angle(),rand_angle()};

		IKS::Homogeneous_T original = puma.fwdkin(joint_angles);

		// Remember to reverse order of joint angles
		std::reverse(joint_angles.begin(), joint_angles.end());
		IKS::Homogeneous_T inverted = puma_reversed.fwdkin(joint_angles);

		is_Pass &= (IKS::inverse_homogeneous_T(original)-inverted).isZero();
	}	

	std::cout << "\n===== Test [Kinematic Inversion]: ";
	if (is_Pass)
	{
		std::cout << "[PASS] =====" << std::endl;
	}
	else
	{
		std::cout << "[FAIL] =====" << std::endl;
	}

	return is_Pass;
}

// Axis 2, 3, 4, parallel
bool ik_test_3P()
{
	// Robot configuration for spherical wrist with second and third axis intersecting
	Eigen::Matrix<double, 3, 6> three_parallel_H;
	three_parallel_H << ez, ex, ex, ex, ez, ex;
	Eigen::Matrix<double, 3, 7> three_parallel_P;
	three_parallel_P << ez, ey, ey, ey, ey, ey + ex, ex;

	IKS::General_6R three_parallel(three_parallel_H, three_parallel_P);

	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(three_parallel.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));

	}

	return evaluate_test("IK three parallel - 2,3,4", three_parallel, ee_poses);
}

// spherical wrist, intersecting axes 1,2, parallel 2,3, intersecting 3,4
bool ik_test_PUMA()
{
	// Robot configuration for spherical wrist with second and third axis intersecting
	Eigen::Matrix<double, 3, 6> puma_H;
	puma_H << ez, -ey, -ey, ez, ey, ez;
	Eigen::Matrix<double, 3, 7> puma_P;
	puma_P << 0.62357 * ez, zv, 0.4318 * ex - 0.16764 * ey, 0.432089 * ez + 0.0381 * ey, zv, zv, 0.05334 * ez;

	IKS::General_6R puma(puma_H, puma_P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(puma.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK spherical wrist - PUMA", puma, ee_poses);
}

// spherical wrist, with the remaining second and third axis intersecting
bool ik_test_SPHERICAL_2_3_I()
{
	// Robot configuration for spherical wrist with second and third axis intersecting
	Eigen::Matrix<double, 3, 6> spherical_intersecting_H;
	spherical_intersecting_H << ex, ez, ey, ez, ex, ey;
	Eigen::Matrix<double, 3, 7> spherical_intersecting_P;
	spherical_intersecting_P << ey, -ey + ez, zv, ey + 2 * ez, zv, zv, 2 * ey;

	IKS::General_6R spherical_intersecting(spherical_intersecting_H, spherical_intersecting_P);
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(spherical_intersecting.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK spherical wrist - Axis 2 intersecting 3", spherical_intersecting, ee_poses);
}

// spherical wrist, with the remaining first and second axis intersecting
bool ik_test_SPHERICAL_1_2_I()
{
	// Robot configuration for spherical wrist with first and second axis intersecting
	Eigen::Matrix<double, 3, 6> spherical_intersecting_H;
	spherical_intersecting_H << ez, ey, ex, ez, ex, ey;
	Eigen::Matrix<double, 3, 7> spherical_intersecting_P;
	spherical_intersecting_P << ez, zv, ez + ey, -ey + 2 * ez, zv, zv, 2 * ey;

	/*  Real-Life example:
		Partial IK: KukaR800FixedQ3
		H << ez, ex, 0.5*ex-0.8660254037844387*ey, ez,  0.5*ex+0.8660254037844387*ey, ez;
		P << 0.33999999999999997*ez, zv, 0.4* ez, 0.4* ez, zv, zv, 0.126*ez;
	*/

	IKS::General_6R spherical_intersecting(spherical_intersecting_H, spherical_intersecting_P);
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(spherical_intersecting.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK spherical wrist - Axis 1 intersecting 2", spherical_intersecting, ee_poses);
}

// spherical wrist, with the remaining first and third axis parallel (Solvable by SP5)
bool ik_test_SPHERICAL_1_3_P()
{
	// Using modified version of Irb6640
	Eigen::Matrix<double, 3, 6> spherical_1_3_H;
	spherical_1_3_H << ey, ez, ey, ez, ex, ey;
	Eigen::Matrix<double, 3, 7> spherical_1_3_P;
	spherical_1_3_P << zv, ex + ez, ez - ex, 2 * ez, zv, zv, 2 * ey;

	IKS::General_6R spherical_1_3(spherical_1_3_H, spherical_1_3_P);
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(spherical_1_3.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK spherical wrist - Axis 1||3", spherical_1_3, ee_poses);
}

// spherical wrist, with the remaining first and second axis parallel
bool ik_test_SPHERICAL_1_2_P()
{
	// Using modified version of Irb6640 where first axis switches place with second and third
	Eigen::Matrix<double, 3, 6> Irb6640_mod_H;
	Irb6640_mod_H << ey, ey, ez, ex, ey, ex;
	Eigen::Matrix<double, 3, 7> Irb6640_mod_P;
	Irb6640_mod_P << zv, 0.32 * ex + 0.78 * ez, 1.075 * ez, 1.1425 * ex + 0.2 * ez, zv, zv, 0.2 * ex;

	IKS::General_6R Irb6640_mod(Irb6640_mod_H, Irb6640_mod_P);
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(Irb6640_mod.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK spherical wrist - Axis 1||2", Irb6640_mod, ee_poses);
}

// spherical wrist, with the remaining second and third axis parallel
// In addition, the third axis is inherently redundant for this configuration
bool ik_test_SPHERICAL_2_3_P_REDUNDANT()
{
	Eigen::Matrix<double, 3, 6> bot_1_H;
	bot_1_H << -ey, -ez, -ez, -ey, -ez, -ey;
	Eigen::Matrix<double, 3, 7> bot_1_P;
	bot_1_P << -0.173679*ey,  0.565723 * ex, 0.397168 * ex, zv, zv, zv,-0.598349*ey;

	IKS::General_6R bot_1(bot_1_H, bot_1_P);
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(bot_1.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK spherical wrist - Axis 2||3", bot_1, ee_poses);
}

// spherical wrist, with the remaining second and third axis parallel
bool ik_test_SPHERICAL_2_3_P()
{
	Eigen::Matrix<double, 3, 6> Irb6640_H;
	Irb6640_H << ez, ey, ey, ex, ey, ex;
	Eigen::Matrix<double, 3, 7> Irb6640_P;
	Irb6640_P << zv, 0.32 * ex + 0.78 * ez, 1.075 * ez, 1.1425 * ex + 0.2 * ez, zv, zv, 0.2 * ex;

	IKS::General_6R Irb6640(Irb6640_H, Irb6640_P);
	/*
	std::vector<IKS::Homogeneous_T> poses;
	for(unsigned i = 0; i < BATCH_SIZE-1; i++)
	{
		IKS::Homogeneous_T ee_pose_Irb6640 = Irb6640.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()});
		poses.push_back(ee_pose_Irb6640);
	}
	IKS::Homogeneous_T ee_pose_Irb6640;
	ee_pose_Irb6640 << -0.239029, -0.949782, 0.201937, -0.199566,
		-0.0690318, -0.190817, -0.979195, -1.89794,
		0.968555, -0.247996, -0.0199543, -0.152028,
		6.93778e-310, 6.93778e-310, 0, 1;*/

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(Irb6640.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK spherical wrist - Axis 2||3", Irb6640, ee_poses);
}

bool ik_test_SPHERICAL()
{
	Eigen::Matrix<double, 3, 6> Spherical_Bot_H;
	Spherical_Bot_H << ey, ez, ey, ex, ey, ex;
	Eigen::Matrix<double, 3, 7> Spherical_Bot_P;
	Spherical_Bot_P << zv, ez + ex, ez + ex, ez + ex, zv, zv, ex;

	IKS::General_6R Spherical_Bot(Spherical_Bot_H, Spherical_Bot_P);
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(Spherical_Bot.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK spherical wrist", Spherical_Bot, ee_poses);
}

bool evaluate_test(const std::string &name_test, const IKS::General_Robot &robot, const std::vector<IKS::Homogeneous_T> &ee_poses)
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
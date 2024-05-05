#include <iostream>
#include <chrono>
#include <string>
#include <eigen3/Eigen/Dense>
#include <random>
#include <vector>

#include "EAIK.h"

#define ERROR_PASS_EPSILON 1e-6
#define BATCH_SIZE 10000

const Eigen::Vector3d zv(0, 0, 0);
const Eigen::Vector3d ex(1, 0, 0);
const Eigen::Vector3d ey(0, 1, 0);
const Eigen::Vector3d ez(0, 0, 1);

// Spherical wrist
bool ik_test_puma();
bool ik_test_IRB6640();
bool ik_test_spherical();
bool ik_test_3P();
bool ik_test_UR5();

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
	ik_test_puma();
	ik_test_IRB6640();
	ik_test_spherical();
	ik_test_3P();
	ik_test_UR5();
	return 0;
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
		ee_poses.push_back(three_parallel.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));

	}

	return evaluate_test("IK UR5 three parallel - 2,3,4", three_parallel, ee_poses);
}

// Axis 2, 3, 4, parallel
bool ik_test_3P()
{
	const Eigen::Vector3d zv(0, 0, 0);
	const Eigen::Vector3d ex(1, 0, 0);
	const Eigen::Vector3d ey(0, 1, 0);
	const Eigen::Vector3d ez(0, 0, 1);

	Eigen::Matrix<double, 3, 6> three_parallel_H;
	three_parallel_H << ez, ex, ex, ex, ez, ex;
	Eigen::Matrix<double, 3, 7> three_parallel_P;
	three_parallel_P << ez, ey, ey, ey, ey, ey + ex, ex;

	EAIK::Robot three_parallel(three_parallel_H, three_parallel_P);

	
	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for(unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(three_parallel.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));

	}

	return evaluate_test("IK three parallel - 2,3,4", three_parallel, ee_poses);
}

// spherical wrist, intersecting axes 1,2, parallel 2,3, intersecting 3,4
bool ik_test_IRB6640()
{
	// Robot configuration for spherical wrist with second and third axis intersecting
	Eigen::Matrix<double, 3, 6> IRB6640_H;
	IRB6640_H << ez, ey, ey, ex, ey, ex;
	Eigen::Matrix<double, 3, 7> IRB6640_P;
	IRB6640_P << 0.32 * ex + 0.78 * ez, 1.075 * ez, 1.1425 * ex + 0.2 * ez, zv, zv, 0.2 * ex, zv;

	EAIK::Robot IRB6640(IRB6640_H, IRB6640_P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(IRB6640.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK spherical wrist - IRB6640", IRB6640, ee_poses);
}

bool ik_test_spherical()
{
	// Robot configuration for spherical wrist with second and third axis intersecting
	Eigen::Matrix<double, 3, 6> H;
	H << ey, ez, ey, ex, ey, ex;
	Eigen::Matrix<double, 3, 7> P;
	P << zv, ex + ez, ex + ez, ex + ez, zv, ex, zv;

	EAIK::Robot spherical(H, P);

	std::vector<IKS::Homogeneous_T> ee_poses;
	ee_poses.reserve(BATCH_SIZE);
	for (unsigned i = 0; i < BATCH_SIZE; i++)
	{
		ee_poses.push_back(spherical.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
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
		ee_poses.push_back(spherical.fwdkin({rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle(), rand_angle()}));
	}

	return evaluate_test("IK spherical wrist - puma", spherical, ee_poses);
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
	for (const auto &pose : ee_poses)
	{
		solution = robot.calculate_IK(pose);
		unsigned non_LS_solutions = 0;

		for (unsigned i = 0; i < solution.Q.size(); i++)
		{
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
	std::cout << "===== Average solution time (nanoseconds): " << time / BATCH_SIZE << " =====" << std::endl;

	return is_passed;
}
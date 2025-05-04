#include <iostream>
#include <chrono>
#include <random>

#include "EAIK.h"
#include "IK_test_utils.h"

// Create a random angle
double rand_angle()
{
	std::random_device rd;
	std::default_random_engine eng(rd());
	std::uniform_real_distribution<double> distr(0, 1);

	double theta = distr(eng) * 2 * M_PI - M_PI;

	return theta;
}

template<class T>
bool evaluate_test(const std::string &name_test, const T &robot, const std::vector<IKS::Homogeneous_T> &ee_poses)
{
	std::cout<<"Kinematic Family: "<<robot.get_kinematic_family()<<std::endl;
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

template bool evaluate_test<EAIK::Robot>(const std::string &name_test, const EAIK::Robot &robot, const std::vector<IKS::Homogeneous_T> &ee_poses);
template bool evaluate_test<IKS::General_Robot>(const std::string &name_test, const IKS::General_Robot &robot, const std::vector<IKS::Homogeneous_T> &ee_poses);

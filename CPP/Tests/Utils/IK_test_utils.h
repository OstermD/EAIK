#ifndef IK_TEST_UTILS_H
#define IK_TEST_UTILS_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include "EAIK.h"

#define ERROR_PASS_EPSILON 1e-5
#define BATCH_SIZE 100

const Eigen::Vector3d zv(0, 0, 0);
const Eigen::Vector3d ex(1, 0, 0);
const Eigen::Vector3d ey(0, 1, 0);
const Eigen::Vector3d ez(0, 0, 1);

double rand_angle();

template<class T>
bool evaluate_test(const std::string &name_test, const T &robot, const std::vector<IKS::Homogeneous_T> &ee_poses);

#endif
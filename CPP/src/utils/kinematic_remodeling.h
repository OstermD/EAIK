#ifndef KINEMATIC_REMODELING
#define KINEMATIC_REMODELING

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <math.h>
#include <tuple>

namespace EAIK
{
    bool do_axes_intersect(const Eigen::Vector3d& h1, const Eigen::Vector3d& h2, const Eigen::Vector3d& p12, const double ZERO_THRESHOLD, const double AXIS_INTERSECT_THRESHOLD);
    bool is_point_on_Axis(const Eigen::Vector3d& h, const Eigen::Vector3d& p0h, const Eigen::Vector3d& p, const double AXIS_INTERSECT_THRESHOLD);
    Eigen::Vector3d calc_intersection(const Eigen::Vector3d& hj, const Eigen::Vector3d& hk, const Eigen::Vector3d& p0j, const Eigen::Vector3d& pkj, const double ZERO_THRESHOLD);
    Eigen::MatrixXd remodel_kinematics(const Eigen::MatrixXd &H, const Eigen::MatrixXd &P, double ZERO_THRESHOLD, const double AXIS_INTERSECT_THRESHOLD);

    // Create kinematic chain that implies series of fixed joint values
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::Matrix<double, 3, 3>> partial_joint_parametrization(const Eigen::MatrixXd &H, const Eigen::MatrixXd &P, std::vector<std::pair<int, double>> fixed_axes, const Eigen::Matrix<double, 3, 3>& R6T=Eigen::Matrix<double, 3, 3>::Identity());
} // namespace EAIK
#endif
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <math.h>

#include "sp.h"
#include "IKS.h"
#include "utils/kinematic_utils.h"

namespace IKS
{
    General_1R::General_1R(const Eigen::Matrix<double, 3, 1> &H, const Eigen::Matrix<double, 3, 2> &P)
        : General_Robot(H,P), H(H), P(P)
    {
    }

    IK_Solution General_1R::calculate_IK(const Homogeneous_T &ee_position_orientation) const
    {
        IK_Solution solution = calculate_position_IK(ee_position_orientation.block<3, 1>(0, 3));

        // 6DOF Problem for 1R is overconstrained -> "Throw away" extraneous orientation solutions
        return enforce_solution_consistency(solution, ee_position_orientation);
    }

    IK_Solution General_1R::calculate_position_IK(const Eigen::Vector3d &ee_position) const
    {
        IK_Solution sol;
        const Eigen::Vector3d p_1t = ee_position - this->P.col(0);
        SP1 sp1(this->P.col(1), p_1t, this->H.col(0));
        sp1.solve();
        sol.Q.push_back({sp1.get_theta()});
        sol.is_LS_vec.push_back(sp1.solution_is_ls());

        return sol;
    }

    IK_Solution General_1R::calculate_orientation_IK(const Eigen::Matrix3d &ee_orientation) const
    {
        IK_Solution sol;
        const Eigen::Vector3d h_n = create_normal_vector(this->H.col(0));
        SP1 sp1(h_n, ee_orientation*h_n, this->H.col(0));
        sp1.solve();
        sol.Q.push_back({sp1.get_theta()});
        sol.is_LS_vec.push_back(sp1.solution_is_ls());
        
        return sol;
    }
};
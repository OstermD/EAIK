#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <math.h>

#include "sp.h"
#include "IKS.h"
#include "utils/kinematic_utils.h"
#include "kinematic_remodeling.h"

namespace IKS
{
    General_2R::General_2R(const Eigen::Matrix<double, 3, 2> &H, const Eigen::Matrix<double, 3, 3> &P)
        : General_Robot(H, P), H(H), P(P)
    {
    }

    IK_Solution General_2R::calculate_IK(const Homogeneous_T &ee_position_orientation) const
    {
        IK_Solution solution = calculate_position_IK(ee_position_orientation.block<3, 1>(0, 3));

        // 6DOF Problem for 2R is overconstrained -> "Throw away" extraneous orientation solutions
        return enforce_solution_consistency(solution, ee_position_orientation);
    }

    IK_Solution General_2R::calculate_position_IK(const Eigen::Vector3d &ee_position) const
    {
        IK_Solution solution;
        const Eigen::Vector3d p_1ee = ee_position - this->P.col(0);
        
        if (EAIK::do_axes_intersect(this->H.col(0), this->H.col(1), this->P.col(1), ZERO_THRESH, ZERO_THRESH))
        {
            // (h1 x h2)(p12)==0) -> h1 intersects h2
            SP2 sp2(p_1ee, this->P.col(2), -this->H.col(0), this->H.col(1));
            sp2.solve();
            const std::vector<double>& theta_1 = sp2.get_theta_1();
            const std::vector<double>& theta_2 = sp2.get_theta_2();

            for(unsigned i = 0; i < theta_1.size(); i++)
            {
                solution.Q.push_back({theta_1.at(i), theta_2.at(i)});
                solution.is_LS_vec.push_back(sp2.solution_is_ls());
            }
        }
        else
        {
            SP3 sp3(this->P.col(2), -this->P.col(1), this->H.col(1), p_1ee.norm());
            sp3.solve();
    
            for (const auto &q2 : sp3.get_theta())
            {
                const Eigen::Matrix3d r_12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();
                SP1 sp1(this->P.col(1) + r_12 * this->P.col(2), p_1ee, this->H.col(0));
                sp1.solve();
                solution.Q.push_back({sp1.get_theta(), q2});
                solution.is_LS_vec.push_back(sp1.solution_is_ls() || sp3.solution_is_ls());
            }    
        }

        return enforce_solution_consistency(solution, ee_position);
    }

    IK_Solution General_2R::calculate_orientation_IK(const Eigen::Matrix3d &ee_orientation) const
    {
        IK_Solution solution;
        if (this->H.col(0).cross(this->H.col(1)).norm() > ZERO_THRESH)
        {
            const Eigen::Vector3d h_n = this->H.col(0).cross(this->H.col(1));
            SP2 sp2(ee_orientation*h_n, h_n, -this->H.col(0), this->H.col(1));
            sp2.solve();

            const std::vector<double> &theta_1 = sp2.get_theta_1();
            const std::vector<double> &theta_2 = sp2.get_theta_2();

            for (unsigned i = 0; i < theta_1.size(); i++)
            {
                solution.Q.push_back({theta_1.at(i), theta_2.at(i)});
                solution.is_LS_vec.push_back(sp2.solution_is_ls());
            }
        }
        else
        {
            // Implicit redundancy
            throw new std::runtime_error("Can not calculate orientation IK for implicitly redundant manipulator");
        }
        
        return enforce_solution_consistency(solution, ee_orientation);
    }
};
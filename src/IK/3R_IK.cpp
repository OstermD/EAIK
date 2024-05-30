#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <math.h>

#include "sp.h"
#include "IKS.h"

namespace IKS
{
    General_3R::General_3R(const Eigen::Matrix<double, 3, 3> &H, const Eigen::Matrix<double, 3, 4> &P)
        : General_Robot(H,P), H(H), P(P)
    {
    }

    IK_Solution General_3R::calculate_IK(const Homogeneous_T &ee_position_orientation) const
    {
        IK_Solution solution;
        const Eigen::Vector3d p_0t = ee_position_orientation.block<3, 1>(0, 3);
        const Eigen::Matrix3d r_03 = ee_position_orientation.block<3, 3>(0, 0);
        
        // Solving 3R kinematics by orientation constraints:
        SP4 sp4(this->H.col(0),
                this->H.col(2),
                this->H.col(1),
                this->H.col(0).transpose() * r_03 * this->H.col(2));
        sp4.solve();

        const std::vector<double> &theta_2 = sp4.get_theta();

        for (const auto &q2 : theta_2)
        {
            const Eigen::Matrix3d r12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();
            SP1 sp1_t1 (r_03*this->H.col(2), r12*this->H.col(2), -this->H.col(0));
            sp1_t1.solve();

            SP1 sp1_t3 (r12.transpose()*this->H.col(0), r_03.transpose()*this->H.col(0), -this->H.col(2));
            sp1_t3.solve();
            solution.Q.push_back({sp1_t1.get_theta(), q2, sp1_t3.get_theta()});
            solution.is_LS_vec.push_back(sp1_t1.solution_is_ls()||sp1_t3.solution_is_ls()||sp4.solution_is_ls());
        }

        // Finding 6DOF Problem for 3R is overconstrained -> "Throw away" extraneous orientation solutions
        for(unsigned i = 0; i < solution.Q.size(); i++)
        {
            if(true)
            {
				IKS::Homogeneous_T result = fwdkin(solution.Q.at(i));
				double error = (result - ee_position_orientation).norm();

                if(error > ZERO_THRESH)
                {
                    solution.is_LS_vec.at(i) = true;
                }
            }
        }
        return solution;
    }
};
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <math.h>

#include "sp.h"
#include "IKS.h"
#include "utils/kinematic_utils.h"

namespace IKS
{
    General_4R::General_4R(const Eigen::Matrix<double, 3, 4> &H, const Eigen::Matrix<double, 3, 5> &P)
        : General_Robot(H,P), H(H), P(P)
    {
    }

    IK_Solution General_4R::calculate_IK(const Homogeneous_T &ee_position_orientation) const
    {
        IK_Solution solution;
        const Eigen::Vector3d p_0t = ee_position_orientation.block<3, 1>(0, 3);
        const Eigen::Matrix3d r_04 = ee_position_orientation.block<3, 3>(0, 0);
        const Eigen::Vector3d p_14 = p_0t - this->P.col(0) - r_04*this->P.col(4);        

        if(this->H.col(2).cross(this->H.col(3)).norm() >= ZERO_THRESH && 
            std::fabs(this->H.col(2).cross(this->H.col(3)).transpose() * this->P.col(3)) < ZERO_THRESH)
        {
            // (h3 x h4)(p34)==0) -> h3 intersects h4
            // And 3 not parallel to 4 (h3 x h4 =/= 0)
        }
        else if (this->H.col(1).cross(this->H.col(2)).norm() >= ZERO_THRESH && 
        std::fabs(this->H.col(1).cross(this->H.col(2)).transpose() * this->P.col(2)) < ZERO_THRESH)// h2xh3(p23) == 0
        {
            // (h2 x h3)(p23)==0) -> h2 intersects h3
            // And 2 not parallel to 3 (h2 x h3 =/= 0)
        }
        else if (this->H.col(0).cross(this->H.col(1)).norm() > ZERO_THRESH)
        {
            if (this->H.col(1).cross(this->H.col(2)).norm() > ZERO_THRESH)
            {
                // h1 =/= h2; h2=/=h3
                SP5 sp5(-this->P.col(1), p_14, this->P.col(2), this->P.col(3), -this->H.col(0), this->H.col(1), this->H.col(2));
                sp5.solve();

                const std::vector<double> &theta_1 = sp5.get_theta_1();
                const std::vector<double> &theta_2 = sp5.get_theta_2();
                const std::vector<double> &theta_3 = sp5.get_theta_3();

                const Eigen::Vector3d h_n = create_normal_vector(this->H.col(3));
                for(unsigned i = 0; i < theta_1.size(); i++)
                {
                    const double& q1 = theta_1.at(i);
                    const double& q2 = theta_2.at(i);
                    const double& q3 = theta_3.at(i);
                    const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();
                    const Eigen::Matrix3d r_12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();
                    const Eigen::Matrix3d r_23 = Eigen::AngleAxisd(q3, this->H.col(2).normalized()).toRotationMatrix();
                    
                    SP1 sp1(h_n, r_23.transpose()*r_12.transpose()*r_01.transpose()*r_04*h_n, this->H.col(3));
                    sp1.solve();

                    solution.Q.push_back({q1, q2, q3, sp1.get_theta()});
                    solution.is_LS_vec.push_back(sp5.solution_is_ls() || sp1.solution_is_ls());
                }

            }
            else
            {
                // h1 =/= h2; h2=h3

            }
        }
        else
        {
            if (this->H.col(1).cross(this->H.col(2)).norm() > ZERO_THRESH)
            {
                // h1 = h2; h2=/=h3
            }
            else
            {
                // Implicit redundancy
            }
        }

        return solution;
    }
};
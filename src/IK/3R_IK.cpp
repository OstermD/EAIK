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
        IK_Solution solution_t_12;
        const Eigen::Vector3d p_0t = ee_position_orientation.block<3, 1>(0, 3);
        const Eigen::Matrix3d r_03 = ee_position_orientation.block<3, 3>(0, 0);
        const Eigen::Vector3d p_03 = p_0t;// - r_03*this->P.col(3);        

        // Checking for parallel Axes
        if (this->H.col(0).cross(this->H.col(1)).norm() < ZERO_THRESH)
        {
            // Axis 1 || Axis 2
            SP3 sp3(this->P.col(2), -this->P.col(1), this->H.col(1), p_03.norm());
            sp3.solve();

            const std::vector<double> &theta_2 = sp3.get_theta();

            for (const auto &q2 : theta_2)
            {
                const Eigen::Matrix3d r12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();
                SP1 sp1(p_03, this->P.col(1)+r12*this->P.col(2), -this->H.col(0));
                sp1.solve();

                solution_t_12.Q.push_back({sp1.get_theta(), q2});
                solution_t_12.is_LS_vec.push_back(sp3.solution_is_ls()||sp1.solution_is_ls());
            }
        }
        else if (std::fabs(this->H.col(0).cross(this->H.col(1)).transpose() * this->P.col(1)) < ZERO_THRESH)
        {
            // (h1 x h2)(p12)==0) -> h1 intersects h2
            SP2 sp2(p_03, this->P.col(2), -this->H.col(0), this->H.col(1));
            sp2.solve();

            for(unsigned i = 0; i < sp2.get_theta_1().size(); i++)
            {
                const double q1 = sp2.get_theta_1().at(i);
                const double q2 = sp2.get_theta_2().at(i);
                solution_t_12.Q.push_back({q1, q2});
                solution_t_12.is_LS_vec.push_back(sp2.solution_is_ls());
            }
        }
        else
        {
            double d = this->H.col(0).transpose()*(p_03-this->P.col(1));
            SP4 sp4(this->H.col(0), this->P.col(2), this->H.col(1), d);
            sp4.solve();

            const std::vector<double> &theta_2 = sp4.get_theta();

            for (const auto &q2 : theta_2)
            {
                const Eigen::Matrix3d r12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();
                SP1 sp1(p_03, this->P.col(1)+r12*this->P.col(2), -this->H.col(0));
                sp1.solve();

                solution_t_12.Q.push_back({sp1.get_theta(), q2});
                solution_t_12.is_LS_vec.push_back(sp4.solution_is_ls()||sp1.solution_is_ls());
            }
        }

        for(unsigned i = 0; i < solution_t_12.Q.size(); i++)
        {
            const double q1 = solution_t_12.Q.at(i).at(0);
            const double q2 = solution_t_12.Q.at(i).at(1);

            const Eigen::Matrix3d r12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();
            const Eigen::Matrix3d r01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();

            const Eigen::Vector3d hn = this->H.col(2).cross(Eigen::Vector3d(1,0,0)); // Vector that's not collinear to h3

            SP1 sp1(hn, r12.transpose()*r01.transpose()*r_03*hn, this->H.col(2));
            sp1.solve();
            solution.Q.push_back({q1, q2, sp1.get_theta()});
            solution.is_LS_vec.push_back(solution_t_12.is_LS_vec.at(i)|sp1.solution_is_ls());
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
                    std::cout<< result<<std::endl;
                    std::cout<< ee_position_orientation<<std::endl;
                    solution.is_LS_vec.at(i) = true;
                }
            }
        }
        return solution;
    }
};
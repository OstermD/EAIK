#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <math.h>

#include "sp.h"
#include "IKS.h"

namespace IKS
{
    General_Robot::General_Robot(const Eigen::Matrix<double, 3, 6> &H, const Eigen::Matrix<double, 3, 7> &P)
        : H(H), P(P)
    {
    }

    IK_Solution General_Robot::calculate_IK(const Homogeneous_T &ee_position_orientation) const
    {
        IK_Solution solution;
        
        const Eigen::Vector3d p_0t = ee_position_orientation.block<3, 1>(0, 3);
        const Eigen::Matrix3d r_06 = ee_position_orientation.block<3, 3>(0, 0);
        const Eigen::Vector3d p_16 = p_0t - this->P.col(0) - r_06 * this->P.col(6);

        // Calculate "Position-IK":
        std::vector<std::vector<double>> position_solutions;

        // Check for parallel axes
        if (this->H.col(0).cross(this->H.col(1)).norm() < ZERO_THRESH &&
            this->H.col(0).cross(this->H.col(2)).norm() < ZERO_THRESH &&
            this->H.col(1).cross(this->H.col(2)).norm() < ZERO_THRESH)
        {
            // h1 || h2 || h3 -> first three axes parallel
        }
        else if (this->H.col(1).cross(this->H.col(2)).norm() < ZERO_THRESH &&
                 this->H.col(1).cross(this->H.col(3)).norm() < ZERO_THRESH &&
                 this->H.col(2).cross(this->H.col(3)).norm() < ZERO_THRESH)
        {
            // h2 || h3 || h4

            std::vector<double> theta_1;
            std::vector<double> theta_5;
            std::vector<bool> is_ls_q1_q5;

            // (h5 x h6)(p56)==0) -> h5 intersects h6
            // And 5 not parallel to 6 (h5 x h6 =/= 0)
            if(this->H.col(4).cross(this->H.col(5)).norm() >= ZERO_THRESH && 
            std::fabs(this->H.col(4).cross(this->H.col(5)).transpose() * this->P.col(5)) < ZERO_THRESH
            )
            { 
                const Eigen::Vector3d p15 = this->P.col(1)+this->P.col(2)+this->P.col(3)+this->P.col(4);
                SP4 sp4_q1(this->H.col(1), p_16, -this->H.col(0), this->H.col(1).transpose()*p15);
                sp4_q1.solve();
                theta_1 = sp4_q1.get_theta();
                std::vector<double> theta_1_new;

                for(unsigned i = 0; i < theta_1.size(); i++)
                {
                    const double& q1 = theta_1.at(i);
                    const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();

                    SP4 sp4_q5(this->H.col(1), this->H.col(5), this->H.col(4), this->H.col(1).transpose()*(r_01.transpose())*r_06*this->H.col(5));
                    sp4_q5.solve();
                    theta_5.insert(theta_5.end(), sp4_q5.get_theta().begin(), sp4_q5.get_theta().end());
                    std::vector<double> q1_temp(sp4_q5.get_theta().size(), q1);
                    theta_1_new.insert(theta_1_new.end(), q1_temp.begin(), q1_temp.end());

                    std::vector<bool> is_ls_temp(sp4_q5.get_theta().size(), sp4_q1.solution_is_ls()||sp4_q5.solution_is_ls());
                    is_ls_q1_q5.insert(is_ls_q1_q5.end(), is_ls_temp.begin(), is_ls_temp.end());
                }
                theta_1 = theta_1_new;
            }
            else
            {
                const double d1 = this->H.col(1).transpose() * (this->P.col(2) + this->P.col(3) + this->P.col(4) + this->P.col(1));
                const double d2 = 0;

                SP6 sp6(this->H.col(1),
                        this->H.col(1),
                        this->H.col(1),
                        this->H.col(1),
                        -this->H.col(0),
                        this->H.col(4),
                        -this->H.col(0),
                        this->H.col(4),
                        p_16,
                        -this->P.col(5),
                        r_06 * this->H.col(5),
                        -this->H.col(5),
                        d1,
                        d2);

                sp6.solve();
                theta_1 = sp6.get_theta_1();
                theta_5 = sp6.get_theta_2();

                is_ls_q1_q5 = std::vector(theta_1.size(), sp6.solution_is_ls());
            }

            for(unsigned i = 0; i < theta_1.size(); i++)
            {
                const double& q1 = theta_1.at(i);
                const double& q5 = theta_5.at(i);

                const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();
                const Eigen::Matrix3d r_45 = Eigen::AngleAxisd(q5, this->H.col(4).normalized()).toRotationMatrix();

                SP1 sp_14(r_45*this->H.col(5), r_01.transpose()*r_06*this->H.col(5), this->H.col(1));
                SP1 sp_q6(r_45.transpose()*this->H.col(1), r_06.transpose()*r_01*this->H.col(1), -this->H.col(5));
                sp_14.solve();

                const Eigen::Matrix3d r_14 = Eigen::AngleAxisd(sp_14.get_theta(), this->H.col(1).normalized()).toRotationMatrix();
                const Eigen::Vector3d d_inner = r_01.transpose()*p_16-this->P.col(1) - r_14*r_45*this->P.col(5) - r_14*this->P.col(4);
                const double d = d_inner.norm();

                SP3 sp3_t3(-this->P.col(3), this->P.col(2), this->H.col(1), d);
                sp3_t3.solve();
                sp_q6.solve();

                const std::vector<double> theta_3 = sp3_t3.get_theta();

                for(const auto& q3 : theta_3)
                {                
                    const Eigen::Matrix3d rot_1 = Eigen::AngleAxisd(q3, this->H.col(1).normalized()).toRotationMatrix();
                    SP1 sp1_q2(this->P.col(2)+rot_1*this->P.col(3), d_inner, this->H.col(1));
                    sp1_q2.solve();

                    double q4 = sp_14.get_theta() - sp1_q2.get_theta() - q3;
                    q4 = std::atan2(std::sin(q4), std::cos(q4)); // Map angle q4 to [-PI,PI)

                    solution.Q.push_back({q1, sp1_q2.get_theta(), q3, q4, q5, sp_q6.get_theta()});
                    solution.is_LS_vec.push_back(is_ls_q1_q5.at(i) || sp1_q2.solution_is_ls() || sp3_t3.solution_is_ls() ||sp_q6.solution_is_ls());
                }
            }
        }
        else if (this->H.col(2).cross(this->H.col(3)).norm() < ZERO_THRESH &&
                 this->H.col(2).cross(this->H.col(4)).norm() < ZERO_THRESH &&
                 this->H.col(3).cross(this->H.col(4)).norm() < ZERO_THRESH)
        {
            // h3 || h4 || h5
        }
        else if (this->H.col(3).cross(this->H.col(4)).norm() < ZERO_THRESH &&
                 this->H.col(3).cross(this->H.col(5)).norm() < ZERO_THRESH &&
                 this->H.col(4).cross(this->H.col(5)).norm() < ZERO_THRESH)
        {
            // h4 || h5 || h6
        }

        return solution;
    }

    Homogeneous_T General_Robot::fwdkin(const std::vector<double> &Q) const
    {
        return fwd_kinematics_ndof(this->H, this->P, Q);
    }
};
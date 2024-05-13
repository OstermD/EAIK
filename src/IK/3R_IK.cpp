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
        const Eigen::Matrix3d r_06 = ee_position_orientation.block<3, 3>(0, 0);
        const Eigen::Vector3d p_16 = p_0t - this->P.col(0);

        // Check for parallel axes
        if (this->H.col(0).cross(this->H.col(1)).norm() < ZERO_THRESH)
        {
            // Axis 1 || Axis 2
            SP4 sp4(this->H.col(0),
                    this->P.col(3),
                    this->H.col(2),
                    this->H.col(0).transpose() * (p_16 - this->P.col(1) - this->P.col(2)));
            sp4.solve();

            const std::vector<double> &theta_3 = sp4.get_theta();
            for (const auto &q3 : theta_3)
            {
                Eigen::Matrix3d rot_3 = Eigen::AngleAxisd(q3, this->H.col(2).normalized()).toRotationMatrix();
                SP3 sp3(p_16,
                        this->P.col(1),
                        -this->H.col(0),
                        (rot_3 * this->P.col(3) + this->P.col(2)).norm());
                sp3.solve();

                const std::vector<double> &theta_1 = sp3.get_theta();

                for (const auto &q1 : theta_1)
                {
                    Eigen::Matrix3d rot_1_inv = Eigen::AngleAxisd(q1, -this->H.col(0).normalized()).toRotationMatrix();
                    SP1 sp1(rot_3 * this->P.col(3) + this->P.col(2),
                            rot_1_inv * p_16 - this->P.col(1),
                            this->H.col(1));
                    sp1.solve();
                    const double &q2 = sp1.get_theta();
                    solution.Q.push_back({q1, q2, q3});
                    solution.is_LS_vec.push_back(sp4.solution_is_ls() || sp3.solution_is_ls() || sp1.solution_is_ls());
                }
            }
        }
        else if (this->H.col(1).cross(this->H.col(2)).norm() < ZERO_THRESH)
        {
            // Axis 2 || Axis 3
            SP4 sp4(this->H.col(1),
                    p_16,
                    -this->H.col(0),
                    this->H.col(1).transpose() * (this->P.col(1) + this->P.col(2) + this->P.col(3)));
            sp4.solve();

            const std::vector<double> &theta_1 = sp4.get_theta();
            for (const auto &q1 : theta_1)
            {
                Eigen::Matrix3d rot_1 = Eigen::AngleAxisd(q1, -this->H.col(0).normalized()).toRotationMatrix();
                SP3 sp3(-this->P.col(3),
                        this->P.col(2),
                        this->H.col(2),
                        (rot_1 * (-p_16) + this->P.col(1)).norm());
                sp3.solve();

                const std::vector<double> &theta_3 = sp3.get_theta();

                for (const auto &q3 : theta_3)
                {
                    Eigen::Matrix3d rot_3 = Eigen::AngleAxisd(q3, this->H.col(2).normalized()).toRotationMatrix();
                    SP1 sp1(-this->P.col(2) - rot_3 * this->P.col(3),
                            rot_1 * (-p_16) + this->P.col(1),
                            this->H.col(1));
                    sp1.solve();
                    const double &q2 = sp1.get_theta();
                    solution.Q.push_back({q1, q2, q3});
                    solution.is_LS_vec.push_back(sp4.solution_is_ls() || sp3.solution_is_ls() || sp1.solution_is_ls());
                }
            }
        }
        // Check for intersecting axes
        else if (std::fabs(this->H.col(0).cross(this->H.col(1)).transpose() * this->P.col(1)) < ZERO_THRESH)
        {
            // (h1 x h2)(p12)==0) -> h1 intersects h2
            SP3 sp3(this->P.col(3),
                    -this->P.col(2),
                    this->H.col(2),
                    p_16.norm());

            sp3.solve();
            const std::vector<double> &theta_3 = sp3.get_theta();
            for (const auto &q3 : theta_3)
            {
                Eigen::Matrix3d rot_3 = Eigen::AngleAxisd(q3, this->H.col(2).normalized()).toRotationMatrix();
                SP2 sp2(p_16,
                        this->P.col(2) + rot_3 * this->P.col(3),
                        -this->H.col(0),
                        this->H.col(1));
                sp2.solve();

                const std::vector<double> &theta_1 = sp2.get_theta_1();
                const std::vector<double> &theta_2 = sp2.get_theta_2();

                if (theta_1.size() != theta_2.size())
                {
                    throw std::runtime_error("Incompatible number of solutions for theta1/2 in SP2!");
                }

                for (unsigned i = 0; i < theta_1.size(); i++)
                {
                    solution.Q.push_back({theta_1.at(i), theta_2.at(i), q3});
                    solution.is_LS_vec.push_back(sp3.solution_is_ls() || sp2.solution_is_ls());
                }
            }
        }
        else if (std::fabs(this->H.col(1).cross(this->H.col(2)).transpose() * this->P.col(2)) < ZERO_THRESH)
        { 
            // (h2 x h3)(p23)==0) -> h2 intersects h3
            SP3 sp3(p_16,
                    this->P.col(1),
                    -this->H.col(0),
                    this->P.col(3).norm());
            sp3.solve();

            const std::vector<double> &theta_1 = sp3.get_theta();
            for (const auto &q1 : theta_1)
            {
                Eigen::Matrix3d rot_1_inv = Eigen::AngleAxisd(q1, -this->H.col(0).normalized()).toRotationMatrix();
                SP2 sp2(rot_1_inv * p_16 - this->P.col(1),
                        this->P.col(3),
                        -this->H.col(1),
                        this->H.col(2));
                sp2.solve();

                const std::vector<double> &theta_2 = sp2.get_theta_1();
                const std::vector<double> &theta_3 = sp2.get_theta_2();

                if (theta_2.size() != theta_3.size())
                {
                    throw std::runtime_error("Incompatible number of solutions for theta1/2 in SP2!");
                }

                for (unsigned i = 0; i < theta_2.size(); i++)
                {
                    solution.Q.push_back({q1, theta_2.at(i), theta_3.at(i)});
                    solution.is_LS_vec.push_back(sp3.solution_is_ls() || sp2.solution_is_ls());
                }
            }
        }
        else
        {
            SP5 position_kinematics(-this->P.col(1),
                                    p_16,
                                    this->P.col(2),
                                    this->P.col(3),
                                    -this->H.col(0),
                                    this->H.col(1),
                                    this->H.col(2));
            position_kinematics.solve();

            const std::vector<double> &q1 = position_kinematics.get_theta_1();
            const std::vector<double> &q2 = position_kinematics.get_theta_2();
            const std::vector<double> &q3 = position_kinematics.get_theta_3();

            if (q1.size() != q2.size() || q2.size() != q3.size())
            {
                throw std::runtime_error("Invalid number of angle combinations gathered from SP5!");
            }

            for (unsigned i = 0; i < q1.size(); i++)
            {
                solution.Q.push_back({q1.at(i), q2.at(i), q3.at(i)});
                solution.is_LS_vec.push_back(position_kinematics.solution_is_ls());
            }
        }

        for(unsigned i = 0; i < solution.Q.size(); i++)
        {
            if(!solution.is_LS_vec.at(i))
            {
				IKS::Homogeneous_T result = fwdkin(solution.Q.at(i));
				double error = (result - ee_position_orientation).norm();
                std::cout<<result<<std::endl;
                std::cout<<ee_position_orientation<<std::endl;
                if(error > ZERO_THRESH)
                {
                    solution.is_LS_vec.at(i) = true;
                }
            }
        }
        return solution;
    }
};
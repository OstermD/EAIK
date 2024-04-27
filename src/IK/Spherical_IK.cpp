#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <math.h>

#include "sp.h"
#include "IKS.h"

namespace IKS
{
    Spherical_Wrist_Robot::Spherical_Wrist_Robot(const Eigen::Matrix<double, 3, 6> &H, const Eigen::Matrix<double, 3, 7> &P)
        : General_Robot(H, P)
    {
    }

    IK_Solution Spherical_Wrist_Robot::calculate_IK(const Homogeneous_T &ee_position_orientation) const
    {
        IK_Solution solution;
        const Eigen::Vector3d p_0t = ee_position_orientation.block<3, 1>(0, 3);
        const Eigen::Matrix3d r_06 = ee_position_orientation.block<3, 3>(0, 0);
        const Eigen::Vector3d p_16 = p_0t - this->P.col(0) - r_06 * this->P.col(6);

        // Calculate "Position-IK":
        std::vector<std::vector<double>> position_solutions;
        std::vector<bool> position_solution_is_LS;

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
                    position_solutions.push_back({q1, q2, q3});
                    position_solution_is_LS.push_back(sp4.solution_is_ls() || sp3.solution_is_ls() || sp1.solution_is_ls());
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
                    position_solutions.push_back({q1, q2, q3});
                    position_solution_is_LS.push_back(sp4.solution_is_ls() || sp3.solution_is_ls() || sp1.solution_is_ls());
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
                    position_solutions.push_back({theta_1.at(i), theta_2.at(i), q3});
                    position_solution_is_LS.push_back(sp3.solution_is_ls() || sp2.solution_is_ls());
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
                    position_solutions.push_back({q1, theta_2.at(i), theta_3.at(i)});
                    position_solution_is_LS.push_back(sp3.solution_is_ls() || sp2.solution_is_ls());
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
                position_solutions.push_back({q1.at(i), q2.at(i), q3.at(i)});
                position_solution_is_LS.push_back(position_kinematics.solution_is_ls());
            }
        }

        // Solve "orientation IK"

        for (unsigned i = 0; i < position_solutions.size(); i++) 
        {
            const auto& p_solution = position_solutions.at(i);
            const double &q1 = p_solution.at(0);
            const double &q2 = p_solution.at(1);
            const double &q3 = p_solution.at(2);

            Eigen::Matrix3d rot_1_inv = Eigen::AngleAxisd(q1, -this->H.col(0).normalized()).toRotationMatrix();
            Eigen::Matrix3d rot_2_inv = Eigen::AngleAxisd(q2, -this->H.col(1).normalized()).toRotationMatrix();
            Eigen::Matrix3d rot_3_inv = Eigen::AngleAxisd(q3, -this->H.col(2).normalized()).toRotationMatrix();

            const Eigen::Matrix3d r_36 = rot_3_inv * rot_2_inv * rot_1_inv * r_06;

            SP4 sp4(this->H.col(3),
                    this->H.col(5),
                    this->H.col(4),
                    this->H.col(3).transpose() * r_36 * this->H.col(5));
            sp4.solve();

            const std::vector<double> &q5s = sp4.get_theta();

            for (const auto &q5 : q5s)
            {
                Eigen::Matrix3d rot_5 = Eigen::AngleAxisd(q5, this->H.col(4).normalized()).toRotationMatrix();
                SP1 sp1_q4(rot_5 * this->H.col(5),
                           r_36 * this->H.col(5),
                           this->H.col(3));
                SP1 sp1_q6(rot_5.transpose() * this->H.col(3),
                           r_36.transpose() * this->H.col(3),
                           -this->H.col(5));
                sp1_q4.solve();
                sp1_q6.solve();
                solution.Q.push_back({q1, q2, q3, sp1_q4.get_theta(), q5, sp1_q6.get_theta()});
                solution.is_LS_vec.push_back(position_solution_is_LS.at(i) || sp4.solution_is_ls() || sp1_q4.solution_is_ls() || sp1_q6.solution_is_ls());
            }
        }

        return solution;
    }

    // Expects Eigen::Matrix<double, 3, N> H axes, Eigen::Matrix<double, 3, N> P offsets and N angles for N joints
    Homogeneous_T fwd_kinematics_ndof(const Eigen::MatrixXd &H, const Eigen::MatrixXd &P, const std::vector<double> &Q)
    {
        // check if H and P are correcly sized
        const int num_axes = Q.size();
        if(H.rows() != 3 || H.cols() != num_axes || P.rows() != 3 || P.cols() != num_axes+1)
        {
            throw std::runtime_error("fwd_kinematics_ndof was given wrong kinematic model!");
        }

        Eigen::Vector3d p = P.col(0);
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        R.setIdentity();

        for (unsigned int i = 0; i < num_axes; i++)
        {
            R = R * Eigen::AngleAxisd(Q.at(i), H.col(i).normalized()).toRotationMatrix();
            p = p + R * P.col(i + 1);
        }

        Homogeneous_T result = Homogeneous_T::Identity();
        result.block<3, 3>(0, 0) = R;
        result.block<3, 1>(0, 3) = p;
        result(3, 3) = 1;
        return result;
    }
};
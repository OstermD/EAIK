#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <math.h>

#include "sp.h"
#include "IKS.h"
#include "utils/kinematic_utils.h"

namespace IKS
{
    General_Robot::General_Robot(const Eigen::MatrixXd &H, const Eigen::MatrixXd &P)
        : H(H), P(P)
    {
    }

    // Expects Eigen::Matrix<double, 3, N> H axes, Eigen::Matrix<double, 3, N> P offsets and N angles for N joints
    Homogeneous_T General_Robot::fwdkin(const std::vector<double> &Q) const
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

    IK_Solution General_Robot::calculate_IK(const Homogeneous_T &ee_position_orientation) const
    {
        throw std::runtime_error("General Robot has no valid IK formulation. Use corresponding specifications for 6R/3R robots or fix joint values for redundant robots");
    }

    bool General_Robot::has_known_decomposition() const
    {
        return false;
    }

    General_6R::General_6R(const Eigen::Matrix<double, 3, 6> &H, const Eigen::Matrix<double, 3, 7> &P)
        : General_Robot(H,P), H(H), P(P)
    {
    }

    bool General_6R::has_known_decomposition() const
    {
        // Check for parallel axes
        if (this->H.col(0).cross(this->H.col(1)).norm() < ZERO_THRESH &&
            this->H.col(0).cross(this->H.col(2)).norm() < ZERO_THRESH &&
            this->H.col(1).cross(this->H.col(2)).norm() < ZERO_THRESH)
        {
            // h1 || h2 || h3 -> first three axes parallel
            // (h5 x h6)(p56)==0) -> h5 intersects h6
            // And 5 not parallel to 6 (h5 x h6 =/= 0)
            if(this->H.col(4).cross(this->H.col(5)).norm() >= ZERO_THRESH && 
            std::fabs(this->H.col(4).cross(this->H.col(5)).transpose() * this->P.col(5)) < ZERO_THRESH)
            {
                return true;
            }
        }
        else if (this->H.col(1).cross(this->H.col(2)).norm() < ZERO_THRESH &&
                 this->H.col(1).cross(this->H.col(3)).norm() < ZERO_THRESH &&
                 this->H.col(2).cross(this->H.col(3)).norm() < ZERO_THRESH)
        {
            // h2 || h3 || h4
            return true;
        }
        else if (this->H.col(2).cross(this->H.col(3)).norm() < ZERO_THRESH &&
                 this->H.col(2).cross(this->H.col(4)).norm() < ZERO_THRESH &&
                 this->H.col(3).cross(this->H.col(4)).norm() < ZERO_THRESH)
        {
            // h3 || h4 || h5
            return true;
        }
        else if (this->H.col(3).cross(this->H.col(4)).norm() < ZERO_THRESH &&
                 this->H.col(3).cross(this->H.col(5)).norm() < ZERO_THRESH &&
                 this->H.col(4).cross(this->H.col(5)).norm() < ZERO_THRESH)
        {
            // h4 || h5 || h6
            if(this->H.col(0).cross(this->H.col(1)).norm() >= ZERO_THRESH && 
            std::fabs(this->H.col(0).cross(this->H.col(1)).transpose() * this->P.col(1)) < ZERO_THRESH)
            {
                return true;
            }
        }

        return false;
    }

    IK_Solution General_6R::calculate_IK(const Homogeneous_T &ee_position_orientation) const
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

            // (h5 x h6)(p56)==0) -> h5 intersects h6
            // And 5 not parallel to 6 (h5 x h6 =/= 0)
            if(this->H.col(4).cross(this->H.col(5)).norm() >= ZERO_THRESH && 
            std::fabs(this->H.col(4).cross(this->H.col(5)).transpose() * this->P.col(5)) < ZERO_THRESH)
            {
                const double d = this->H.col(0).transpose()*(p_16 - this->P.col(1) - this->P.col(2) - this->P.col(3));
                SP4 sp4_t4(this->H.col(0),this->P.col(4), this->H.col(3), d);
                sp4_t4.solve();

                for(const auto& q4 : sp4_t4.get_theta())
                {
                    const Eigen::Matrix3d r_34 = Eigen::AngleAxisd(q4, this->H.col(3).normalized()).toRotationMatrix();
                    SP4 sp4_t6(this->H.col(4), r_06.transpose()*this->H.col(0), this->H.col(5), this->H.col(4).transpose()*r_34.transpose()*this->H.col(0));
                    sp4_t6.solve();

                    for(const auto& q6 : sp4_t6.get_theta())
                    {
                        const Eigen::Matrix3d r_56 = Eigen::AngleAxisd(q6, this->H.col(5).normalized()).toRotationMatrix();
                    
                        SP4 sp4_t03(this->H.col(3), r_06*r_56.transpose()*this->H.col(4), -this->H.col(0), this->H.col(3).transpose()*this->H.col(4));
                        sp4_t03.solve();

                        for(const auto& q03 : sp4_t03.get_theta())
                        {
                            const Eigen::Matrix3d r_03 = Eigen::AngleAxisd(q03, this->H.col(0).normalized()).toRotationMatrix();
                            const Eigen::Vector3d hn = create_normal_vector(this->H.col(4));

                            SP1 sp1_t5(r_56*hn, r_34.transpose()*r_03.transpose()*r_06*hn, this->H.col(4));
                            sp1_t5.solve();
                            
                            const Eigen::Vector3d delta = this->P.col(3) + r_34*this->P.col(4);

                            SP3 sp3_t2(this->P.col(2), -this->P.col(1), this->H.col(1), (p_16-r_03*delta).norm());
                            sp3_t2.solve();

                            for(const auto& q2 : sp3_t2.get_theta())
                            {
                                const Eigen::Matrix3d r_12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();
                                SP1 sp1_t1(this->P.col(1)+r_12*this->P.col(2), p_16-r_03*delta, this->H.col(0));
                                sp1_t1.solve();
                                const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(sp1_t1.get_theta(), this->H.col(0).normalized()).toRotationMatrix();
                                
                                const Eigen::Vector3d hn_3 = create_normal_vector(this->H.col(2));
                                SP1 sp1_t3(hn_3, r_12.transpose()*r_01.transpose()*r_03*hn_3, this->H.col(2));
                                sp1_t3.solve();

                                solution.Q.push_back({sp1_t1.get_theta(), q2, sp1_t3.get_theta(), q4, sp1_t5.get_theta(), q6});
                                solution.is_LS_vec.push_back(sp1_t1.solution_is_ls() || sp3_t2.solution_is_ls() || sp1_t3.solution_is_ls() || sp4_t4.solution_is_ls() || sp1_t5.solution_is_ls() || sp4_t6.solution_is_ls());
                            }
                        }
                    }
                }
            }
            else
            {
                throw std::runtime_error("This manipulator type (Axis 1,2,3 parallel with no additional parallel axes) is not yet solvable by EAIK.");
            }
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
            // Reverse kinematic chain and calculate IK for robot with h2 || h3 || h4
            const auto&[H_reversed, P_reversed] = reverse_kinematic_chain(this->H, this->P);
            General_6R reversed_robot (H_reversed, P_reversed);

            solution = reversed_robot.calculate_IK(inverse_homogeneous_T(ee_position_orientation));

            reverse_vector_second_dimension(solution.Q);
        }
        else if (this->H.col(3).cross(this->H.col(4)).norm() < ZERO_THRESH &&
                 this->H.col(3).cross(this->H.col(5)).norm() < ZERO_THRESH &&
                 this->H.col(4).cross(this->H.col(5)).norm() < ZERO_THRESH)
        {
            // h4 || h5 || h6
            // Reverse kinematic chain and calculate IK for robot with h1 || h2 || h3
            const auto&[H_reversed, P_reversed] = reverse_kinematic_chain(this->H, this->P);
            General_6R reversed_robot (H_reversed, P_reversed);

            solution = reversed_robot.calculate_IK(inverse_homogeneous_T(ee_position_orientation));
            reverse_vector_second_dimension(solution.Q);
        }

        if (solution.Q.empty())
        {
            throw std::runtime_error("No valid solution found for given end-effector position and orientation.");
        }
        
        return solution;
    }
};
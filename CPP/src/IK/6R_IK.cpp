#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <math.h>
#include <memory>

#include "sp.h"
#include "IKS.h"
#include "utils/kinematic_utils.h"
#include "kinematic_remodeling.h"


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

    bool General_Robot::is_spherical() const
    {
        return false;
    }

    IK_Solution General_Robot::enforce_solution_consistency(IK_Solution inconsistent_solution, const Homogeneous_T& desiredT, const double& error_threshold) const
    {
        for(unsigned i = 0; i < inconsistent_solution.Q.size(); ++i)
        {
            if(!inconsistent_solution.is_LS_vec.at(i))
            {
                IKS::Homogeneous_T result = fwdkin(inconsistent_solution.Q.at(i));
				double error = (result - desiredT).norm();

                if(error > error_threshold)
                {
                    inconsistent_solution.is_LS_vec.at(i) = true;
                }
            }
        }

        return inconsistent_solution;
    }

    IK_Solution General_Robot::enforce_solution_consistency(IK_Solution inconsistent_solution, const Eigen::Vector3d& desiredPosition, const double& error_threshold) const
    {
        for(unsigned i = 0; i < inconsistent_solution.Q.size(); ++i)
        {
            if(!inconsistent_solution.is_LS_vec.at(i))
            {
                IKS::Homogeneous_T result = fwdkin(inconsistent_solution.Q.at(i));
                const Eigen::Vector3d& resultPosition = result.block<3, 1>(0,3);
				double error = (resultPosition - desiredPosition).norm();

                if(error > error_threshold)
                {
                    inconsistent_solution.is_LS_vec.at(i) = true;
                }
            }
        }

        return inconsistent_solution;
    }

    IK_Solution General_Robot::enforce_solution_consistency(IK_Solution inconsistent_solution, const Eigen::Matrix3d& desiredOrientation, const double& error_threshold) const
    {
        for(unsigned i = 0; i < inconsistent_solution.Q.size(); ++i)
        {
            if(!inconsistent_solution.is_LS_vec.at(i))
            {
                IKS::Homogeneous_T result = fwdkin(inconsistent_solution.Q.at(i));
                const Eigen::Matrix3d& resultOrientation = result.block<3, 3>(0,0);
				double error = (resultOrientation - desiredOrientation).norm();

                if(error > error_threshold)
                {
                    inconsistent_solution.is_LS_vec.at(i) = true;
                }
            }
        }

        return inconsistent_solution;
    }

    General_6R::General_6R(const Eigen::Matrix<double, 3, 6> &H, const Eigen::Matrix<double, 3, 7> &P)
        : General_Robot(H,P), H(H), P(P)
    {
        this->kinematicClass = determine_Kinematic_Class();
    }

    std::string General_6R::get_kinematic_family() const
    {
        switch (kinematicClass)
        {
        case KinematicClass::THREE_INNER_PARALLEL:
            return std::string("6R-THREE_INNER_PARALLEL");
        case KinematicClass::THREE_PARALLEL_TWO_INTERSECTING:
            return std::string("6R-THREE_PARALLEL_TWO_INTERSECTING");
        case KinematicClass::SPHERICAL_FIRST_TWO_PARALLEL:
            return std::string("6R-SPHERICAL_FIRST_TWO_PARALLEL");
        case KinematicClass::SPHERICAL_SECOND_TWO_PARALLEL:
            return std::string("6R-SPHERICAL_SECOND_TWO_PARALLEL");
        case KinematicClass::SPHERICAL_FIRST_TWO_INTERSECTING:
            return std::string("6R-SPHERICAL_FIRST_TWO_INTERSECTING");
        case KinematicClass::SPHERICAL_SECOND_TWO_INTERSECTING:
            return std::string("6R-SPHERICAL_SECOND_TWO_INTERSECTING");
        case KinematicClass::SPHERICAL_NO_PARALLEL_NO_INTERSECTING:
            return std::string("6R-SPHERICAL_NO_PARALLEL_NO_INTERSECTING");
        case KinematicClass::REVERSED:
            return reversed_Robot_ptr ? reversed_Robot_ptr->get_kinematic_family() : "6R-Unknown Kinematic Class";
        default:
            return std::string("6R-Unknown Kinematic Class");
        }
    }

    General_6R::KinematicClass General_6R::determine_Kinematic_Class()
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
                return KinematicClass::THREE_PARALLEL_TWO_INTERSECTING;
            }
        }

        if (this->H.col(1).cross(this->H.col(2)).norm() < ZERO_THRESH &&
                 this->H.col(1).cross(this->H.col(3)).norm() < ZERO_THRESH &&
                 this->H.col(2).cross(this->H.col(3)).norm() < ZERO_THRESH)
        {
            // h2 || h3 || h4
            return KinematicClass::THREE_INNER_PARALLEL;
        }
        
        if (this->H.col(2).cross(this->H.col(3)).norm() < ZERO_THRESH &&
                 this->H.col(2).cross(this->H.col(4)).norm() < ZERO_THRESH &&
                 this->H.col(3).cross(this->H.col(4)).norm() < ZERO_THRESH)
        {
            // h3 || h4 || h5
            const auto&[H_reversed, P_reversed] = reverse_kinematic_chain(this->H, this->P);
            const Eigen::MatrixXd P_reversed_remodeled = EAIK::remodel_kinematics(H_reversed, P_reversed, ZERO_THRESH, ZERO_THRESH);

            this->reversed_Robot_ptr = std::make_unique<General_6R>(H_reversed, P_reversed_remodeled);
            return KinematicClass::REVERSED;
        }
        
        if (this->H.col(3).cross(this->H.col(4)).norm() < ZERO_THRESH &&
                 this->H.col(3).cross(this->H.col(5)).norm() < ZERO_THRESH &&
                 this->H.col(4).cross(this->H.col(5)).norm() < ZERO_THRESH)
        {
            // h4 || h5 || h6
            // (h1 x h2)(p12)==0) -> h1 intersects h2
            // And 1 not parallel to 2 (h1 x h2 =/= 0)
            if(this->H.col(0).cross(this->H.col(1)).norm() >= ZERO_THRESH && 
            std::fabs(this->H.col(0).cross(this->H.col(1)).transpose() * this->P.col(1)) < ZERO_THRESH)
            {
                // h3 || h4 || h5
                const auto&[H_reversed, P_reversed] = reverse_kinematic_chain(this->H, this->P);
                this->reversed_Robot_ptr = std::make_unique<General_6R>(H_reversed, P_reversed);
                return KinematicClass::REVERSED;
            }
        }

        //
        //  Spherical Manipulators
        //

        // Check for spherical wrist
        if (EAIK::do_axes_intersect(H.col(3), H.col(4), P.col(4), ZERO_THRESH, ZERO_THRESH))
        {
            const Eigen::Vector3d p04 = P.block<3,4>(0,0).rowwise().sum();
            const Eigen::Vector3d intersection = EAIK::calc_intersection(H.col(3), H.col(4), p04,P.col(4), ZERO_THRESH);

            if(EAIK::is_point_on_Axis(H.col(5), p04+P.col(4)+P.col(5), intersection, ZERO_THRESH))
            {
                // Check for parallel axes
                if (this->H.col(0).cross(this->H.col(1)).norm() < ZERO_THRESH)
                {
                    // 1 || 2
                    return KinematicClass::SPHERICAL_FIRST_TWO_PARALLEL;
                }

                // Check for parallel axes
                if (this->H.col(1).cross(this->H.col(2)).norm() < ZERO_THRESH)
                {
                    // 2 || 3
                    return KinematicClass::SPHERICAL_SECOND_TWO_PARALLEL;
                }

                if (std::fabs(this->H.col(0).cross(this->H.col(1)).transpose() * this->P.col(1)) < ZERO_THRESH)
                {
                    // (h1 x h2)(p12)==0) -> h1 intersects h2
                    return KinematicClass::SPHERICAL_FIRST_TWO_INTERSECTING;
                }

                if (std::fabs(this->H.col(1).cross(this->H.col(2)).transpose() * this->P.col(2)) < ZERO_THRESH)
                {
                    // (h2 x h3)(p23)==0) -> h2 intersects h3
                    return KinematicClass::SPHERICAL_SECOND_TWO_INTERSECTING;
                }

                return KinematicClass::SPHERICAL_NO_PARALLEL_NO_INTERSECTING;
            }

        }
        
        if (EAIK::do_axes_intersect(H.col(0), H.col(1), P.col(1), ZERO_THRESH, ZERO_THRESH))
        {
            const Eigen::Vector3d intersection = EAIK::calc_intersection(H.col(0), H.col(1), P.col(0), P.col(1), ZERO_THRESH);

            if(EAIK::is_point_on_Axis(H.col(2), P.col(0)+P.col(1)+P.col(2), intersection, ZERO_THRESH))
            {
                const auto&[H_reversed, P_reversed] = reverse_kinematic_chain(this->H, this->P);
                const Eigen::MatrixXd P_reversed_remodeled = EAIK::remodel_kinematics(H_reversed, P_reversed, ZERO_THRESH, ZERO_THRESH);

                this->reversed_Robot_ptr = std::make_unique<General_6R>(H_reversed, P_reversed_remodeled);
                // Spherical Wrist at the base of the robot
                return KinematicClass::REVERSED;
            }
        }
        return KinematicClass::UNKNOWN;   
    }

    bool General_6R::has_known_decomposition() const
    {
        return this->kinematicClass != KinematicClass::UNKNOWN;
    }

    bool General_6R::is_spherical() const
    {
        if (this->kinematicClass==KinematicClass::REVERSED)
        {
            return this->reversed_Robot_ptr->is_spherical();
        }

        return this->kinematicClass >= KinematicClass::SPHERICAL_FIRST_TWO_PARALLEL 
                && this->kinematicClass <= KinematicClass::SPHERICAL_NO_PARALLEL_NO_INTERSECTING;
    }

    IK_Solution General_6R::calculate_IK(const Homogeneous_T &ee_position_orientation) const
    {
        IK_Solution solution;

        const Eigen::Vector3d p_0t = ee_position_orientation.block<3, 1>(0, 3);
        const Eigen::Matrix3d r_06 = ee_position_orientation.block<3, 3>(0, 0);
        const Eigen::Vector3d p_16 = p_0t - this->P.col(0) - r_06 * this->P.col(6);

        // Calculate "Position-IK":
        std::vector<std::vector<double>> position_solutions;

        switch(this->kinematicClass)
        {
            case KinematicClass::THREE_PARALLEL_TWO_INTERSECTING:
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
                break;
            };
            case KinematicClass::THREE_INNER_PARALLEL:
            {
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
                break;
            };
            case KinematicClass::SPHERICAL_FIRST_TWO_PARALLEL:
            {
                // Calculate "Position-IK":
                std::vector<std::vector<double>> position_solutions;
                std::vector<bool> position_solution_is_LS;

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
                return calculate_Spherical_Wrist_Orientation_Kinematics(position_solutions, position_solution_is_LS, r_06);
                break;
            };

            case KinematicClass::SPHERICAL_SECOND_TWO_PARALLEL:
            {
                // Calculate "Position-IK":
                std::vector<std::vector<double>> position_solutions;
                std::vector<bool> position_solution_is_LS;

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
                return calculate_Spherical_Wrist_Orientation_Kinematics(position_solutions, position_solution_is_LS, r_06);
                break;
            };

            case KinematicClass::SPHERICAL_FIRST_TWO_INTERSECTING:
            {
                // Calculate "Position-IK":
                std::vector<std::vector<double>> position_solutions;
                std::vector<bool> position_solution_is_LS;

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
                return calculate_Spherical_Wrist_Orientation_Kinematics(position_solutions, position_solution_is_LS, r_06);
                break;
            };

            case KinematicClass::SPHERICAL_SECOND_TWO_INTERSECTING:
            {
                // Calculate "Position-IK":
                std::vector<std::vector<double>> position_solutions;
                std::vector<bool> position_solution_is_LS;

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
                return calculate_Spherical_Wrist_Orientation_Kinematics(position_solutions, position_solution_is_LS, r_06);
                break;
            };

            case KinematicClass::SPHERICAL_NO_PARALLEL_NO_INTERSECTING:
            {
                // Calculate "Position-IK":
                std::vector<std::vector<double>> position_solutions;
                std::vector<bool> position_solution_is_LS;

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
                return calculate_Spherical_Wrist_Orientation_Kinematics(position_solutions, position_solution_is_LS, r_06);
                break;
            };

            case KinematicClass::REVERSED:
            {
                if(reversed_Robot_ptr)
                {
                    // Use reversed kinematic chain and calculate IK for robot with h2 || h3 || h4
                    solution = this->reversed_Robot_ptr->calculate_IK(inverse_homogeneous_T(ee_position_orientation));
                    reverse_vector_second_dimension(solution.Q);
                }
                else
                {
                    throw std::runtime_error("Reversed kinematic chain was not initialized! Please open an issue on our GitHub repository.");
                }
                
                break;
            };
            default:
                std::cerr<< "The choosen manipulator has no known subproblem decomposition! The resulting solutions will be empty."<<std::endl;
        }

        return solution;
    }

    IK_Solution General_6R::calculate_Spherical_Wrist_Orientation_Kinematics(const std::vector<std::vector<double>>& position_solutions, const std::vector<bool>& position_solution_is_LS, const Eigen::Matrix3d& r_06) const
    {
        IK_Solution solution;

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
};
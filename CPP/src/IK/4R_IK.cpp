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
    General_4R::General_4R(const Eigen::Matrix<double, 3, 4> &H, const Eigen::Matrix<double, 3, 5> &P)
        : General_Robot(H,P), H(H), P(P)
    {
        kinematicClass = determine_Kinematic_Class();
    }

    General_4R::KinematicClass General_4R::determine_Kinematic_Class()
    {
        if (this->H.col(0).cross(this->H.col(1)).norm() >= ZERO_THRESH && 
        std::fabs(this->H.col(0).cross(this->H.col(1)).transpose() * this->P.col(1)) < ZERO_THRESH)
        {
            // (h1 x h2)(p12)==0) -> h1 intersects h2
            // And 1 not parallel to 2 (h1 x h2 =/= 0)
            const Eigen::Vector3d intersection = EAIK::calc_intersection(H.col(0), H.col(1), P.col(0), P.col(1), ZERO_THRESH);

            if(EAIK::is_point_on_Axis(H.col(2), P.col(0)+P.col(1)+P.col(2), intersection, ZERO_THRESH))
            {
                const auto&[H_reversed, P_reversed] = reverse_kinematic_chain(this->H, this->P);
                const Eigen::MatrixXd P_reversed_remodeled = EAIK::remodel_kinematics(H_reversed, P_reversed, ZERO_THRESH, ZERO_THRESH);

                this->reversed_Robot_ptr = std::make_unique<General_4R>(H_reversed, P_reversed_remodeled);
                // Spherical Wrist at the base of the robot
                return KinematicClass::REVERSED;
            }
            else if(this->H.col(2).cross(this->H.col(3)).norm() >= ZERO_THRESH && 
            std::fabs(this->H.col(2).cross(this->H.col(3)).transpose() * this->P.col(3)) < ZERO_THRESH)
            {
                // (h3 x h4)(p34)==0) -> h3 intersects h4
                // And 3 not parallel to 4 (h3 x h4 =/= 0)
                return KinematicClass::FIRST_TWO_LAST_TWO_INTERSECTING;
            }

            // Same as if h3 intersects with h4 -> Use reversed kinematics
            const auto&[H_reversed, P_reversed] = reverse_kinematic_chain(this->H, this->P);
            const Eigen::MatrixXd P_reversed_remodeled = EAIK::remodel_kinematics(H_reversed, P_reversed, ZERO_THRESH, ZERO_THRESH);

            this->reversed_Robot_ptr = std::make_unique<General_4R>(H_reversed, P_reversed_remodeled);
            return KinematicClass::REVERSED;
        }

        // Spherical wrist at end of manipulator
        if (this->H.col(1).cross(this->H.col(2)).norm() >= ZERO_THRESH && 
            std::fabs(this->H.col(1).cross(this->H.col(2)).transpose() * this->P.col(2)) < ZERO_THRESH)
        {
            const Eigen::Vector3d p02 = P.block<3,2>(0,0).rowwise().sum();
            const Eigen::Vector3d intersection = EAIK::calc_intersection(H.col(1), H.col(2), p02,P.col(2), ZERO_THRESH);

            if(EAIK::is_point_on_Axis(H.col(3), p02+P.col(2)+P.col(3), intersection, ZERO_THRESH))
            {
                return KinematicClass::SPHERICAL_WRIST;
            }

            return KinematicClass::SECOND_THIRD_INTERSECTING;
        }

        if(this->H.col(2).cross(this->H.col(3)).norm() >= ZERO_THRESH && 
        std::fabs(this->H.col(2).cross(this->H.col(3)).transpose() * this->P.col(3)) < ZERO_THRESH)
        {
            // (h3 x h4)(p34)==0) -> h3 intersects h4
            // And 3 not parallel to 4 (h3 x h4 =/= 0)
            return KinematicClass::THIRD_FOURTH_INTERSECTING;
        }
    
        if (this->H.col(0).cross(this->H.col(1)).norm() > ZERO_THRESH)
        {
            // h1 =/= h2 
            if (this->H.col(1).cross(this->H.col(2)).norm() > ZERO_THRESH)
            {
                // h2 =/= h3 
                if (this->H.col(2).cross(this->H.col(3)).norm() < ZERO_THRESH)
                {
                    // h3 == h4
                    const auto&[H_reversed, P_reversed] = reverse_kinematic_chain(this->H, this->P);
                    const Eigen::MatrixXd P_reversed_remodeled = EAIK::remodel_kinematics(H_reversed, P_reversed, ZERO_THRESH, ZERO_THRESH);
    
                    this->reversed_Robot_ptr = std::make_unique<General_4R>(H_reversed, P_reversed_remodeled);
                    return KinematicClass::REVERSED;
                }
                return KinematicClass::NONE_PARALLEL_NONE_INTERSECTING;
            }
            // h2 == h3 
            return KinematicClass::SECOND_THIRD_PARALLEL;   
        }
        else if(this->H.col(1).cross(this->H.col(2)).norm() > ZERO_THRESH)
        {
            // h1 == h2 , h2 =/= h3
            return KinematicClass::FIRST_SECOND_PARALLEL;
        }
        else
        {
            // h1 == h2 , h2 == h3 -> h3 =/= h4 for non-redundant manipulator -> reverse
            const auto&[H_reversed, P_reversed] = reverse_kinematic_chain(this->H, this->P);
            const Eigen::MatrixXd P_reversed_remodeled = EAIK::remodel_kinematics(H_reversed, P_reversed, ZERO_THRESH, ZERO_THRESH);

            this->reversed_Robot_ptr = std::make_unique<General_4R>(H_reversed, P_reversed_remodeled);
            return KinematicClass::REVERSED;
        }

        return KinematicClass::UNKNOWN;     
    }

    IK_Solution General_4R::calculate_IK(const Homogeneous_T &ee_position_orientation) const
    {
        IK_Solution solution;
        const Eigen::Vector3d p_0t = ee_position_orientation.block<3, 1>(0, 3);
        const Eigen::Matrix3d r_04 = ee_position_orientation.block<3, 3>(0, 0);
        const Eigen::Vector3d p_14 = p_0t - this->P.col(0) - r_04*this->P.col(4);        

        switch(this->kinematicClass)
        {
            case KinematicClass::THIRD_FOURTH_INTERSECTING:
            {
                // (h3 x h4)(p34)==0) -> h3 intersects h4
                // And 3 not parallel to 4 (h3 x h4 =/= 0)
                SP3 sp3_q1(p_14, this->P.col(1), -this->H.col(0), this->P.col(2).norm());
                sp3_q1.solve();

                for(const auto& q1 : sp3_q1.get_theta())
                {
                    const Eigen::Matrix3d r_10 = Eigen::AngleAxisd(q1, -this->H.col(0).normalized()).toRotationMatrix();
                    SP3 sp3_q2(this->P.col(2), -this->P.col(1), this->H.col(1), p_14.norm());
                    sp3_q2.solve();

                    for(const auto& q2 : sp3_q2.get_theta())
                    {
                        const Eigen::Matrix3d r_21 = Eigen::AngleAxisd(q2, -this->H.col(1).normalized()).toRotationMatrix();

                        const Eigen::Vector3d hn = create_normal_vector(this->H.col(2));
                        SP2 sp2(r_21*r_10*r_04*hn, hn, -this->H.col(2), this->H.col(3));
                        sp2.solve();

                        const std::vector<double>& theta_3 = sp2.get_theta_1();
                        const std::vector<double>& theta_4 = sp2.get_theta_2();

                        for(unsigned i = 0; i < theta_3.size(); ++i)
                        {
                            solution.Q.push_back({q1, q2, theta_3.at(i), theta_4.at(i)});
                            solution.is_LS_vec.push_back(sp3_q1.solution_is_ls() || sp3_q2.solution_is_ls() || sp2.solution_is_ls());
                        }
                    }
                }
                break;
            };
            case KinematicClass::SECOND_THIRD_INTERSECTING:
            {
                SP3 sp3(this->P.col(1), p_14, this->H.col(0), this->P.col(3).norm());
                sp3.solve();

                for(const auto& q1 : sp3.get_theta())
                {
                    const Eigen::Matrix3d r_10 = Eigen::AngleAxisd(q1, -this->H.col(0).normalized()).toRotationMatrix();
                    SP2 sp2(r_10*p_14 - this->P.col(1), this->P.col(3), -this->H.col(1), this->H.col(2));
                    sp2.solve();

                    const std::vector<double>& theta_2 = sp2.get_theta_1();
                    const std::vector<double>& theta_3 = sp2.get_theta_2();
                    
                    const Eigen::Vector3d h_n = create_normal_vector(this->H.col(3));
                    for(unsigned i = 0; i < theta_2.size(); ++i)
                    {
                        const double& q2 = theta_2.at(i);
                        const double& q3 = theta_3.at(i);

                        const Eigen::Matrix3d r_12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();
                        const Eigen::Matrix3d r_23 = Eigen::AngleAxisd(q3, this->H.col(2).normalized()).toRotationMatrix();

                        SP1 sp1(h_n, r_04.transpose()*r_10.transpose()*r_12*r_23*h_n, -this->H.col(3));
                        sp1.solve();

                        solution.Q.push_back({q1, q2, q3, sp1.get_theta()});
                        solution.is_LS_vec.push_back(sp3.solution_is_ls() || sp2.solution_is_ls() || sp1.solution_is_ls());
                    }
                }

                break;
            };
            case KinematicClass::FIRST_SECOND_PARALLEL:
            {
                // h1 = h2; h2=/=h3
                SP4 sp4_q3(this->H.col(0), this->P.col(3), this->H.col(2), this->H.col(0).transpose()*(p_14 - this->P.col(1) - this->P.col(2)));
                sp4_q3.solve();

                for(const auto& q3 : sp4_q3.get_theta())
                {
                    const Eigen::Matrix3d r_23 = Eigen::AngleAxisd(q3, this->H.col(2).normalized()).toRotationMatrix();
                    SP3 sp3_q1(p_14, this->P.col(1), -this->H.col(0), (this->P.col(2)+r_23*this->P.col(3)).norm());
                    sp3_q1.solve();

                    for(const auto& q1 : sp3_q1.get_theta())
                    {
                        const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();
                        SP1 sp1_q2(this->P.col(2)+r_23*this->P.col(3), r_01.transpose()*p_14-this->P.col(1), this->H.col(1));
                        sp1_q2.solve();

                        const Eigen::Matrix3d r_12 = Eigen::AngleAxisd(sp1_q2.get_theta(), this->H.col(1).normalized()).toRotationMatrix();
                        const Eigen::Vector3d hn = create_normal_vector(this->H.col(3));
                        SP1 sp1_q4(hn, r_04.transpose()*r_01*r_12*r_23*hn, -this->H.col(3));
                        sp1_q4.solve();

                        solution.Q.push_back({q1, sp1_q2.get_theta(), q3, sp1_q4.get_theta()});
                        solution.is_LS_vec.push_back(sp4_q3.solution_is_ls() || sp3_q1.solution_is_ls() || sp1_q2.solution_is_ls() || sp1_q4.solution_is_ls());
                    }
                }
                break;
            };
            case KinematicClass::SECOND_THIRD_PARALLEL:
            {
                // h1 =/= h2; h2=h3
                SP4 sp4(this->H.col(1), p_14, -this->H.col(0), this->H.col(1).transpose()*(this->P.col(1) + this->P.col(2) + this->P.col(3)));
                sp4.solve();

                for(const auto& q1 : sp4.get_theta())
                {
                    const Eigen::Matrix3d r_10 = Eigen::AngleAxisd(q1, -this->H.col(0).normalized()).toRotationMatrix();
                    SP3 sp3(this->P.col(3), -this->P.col(2), this->H.col(2), (r_10*p_14-this->P.col(1)).norm());
                    sp3.solve();

                    for(const auto& q3 : sp3.get_theta())
                    {
                        const Eigen::Matrix3d r_23 = Eigen::AngleAxisd(q3, this->H.col(2).normalized()).toRotationMatrix();
                        SP1 sp1_q2(this->P.col(2) + r_23*this->P.col(3), r_10*p_14-this->P.col(1), this->H.col(1));
                        sp1_q2.solve();

                        const double q2 = sp1_q2.get_theta();
                        const Eigen::Matrix3d r_12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();

                        const Eigen::Vector3d hn = create_normal_vector(this->H.col(3));
                        SP1 sp1_q4(hn, r_04.transpose()*r_10.transpose()*r_12*r_23*hn, -this->H.col(3));
                        sp1_q4.solve();

                        solution.Q.push_back({q1, q2, q3, sp1_q4.get_theta()});
                        solution.is_LS_vec.push_back(sp4.solution_is_ls() || sp3.solution_is_ls() || sp1_q2.solution_is_ls() || sp1_q4.solution_is_ls());
                    }
                }
                break;
            };
            case KinematicClass::NONE_PARALLEL_NONE_INTERSECTING:
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
                break;
            };
            case KinematicClass::FIRST_TWO_LAST_TWO_INTERSECTING:
            {
                SP2 sp2_q12(p_14, this->P.col(2), -this->H.col(0), this->H.col(1));
                sp2_q12.solve();

                const std::vector<double> &theta_1 = sp2_q12.get_theta_1();
                const std::vector<double> &theta_2 = sp2_q12.get_theta_2();
                
                for(unsigned i = 0; i < theta_1.size(); ++i)
                {
                    const double& q1 = theta_1.at(i);
                    const double& q2 = theta_2.at(i);
                    const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();
                    const Eigen::Matrix3d r_12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();

                    const Eigen::Vector3d h_n = this->H.col(3).cross(r_04.transpose()*r_01*r_12*this->H.col(2));
                    SP2 sp2_q34(r_12.transpose()*r_01.transpose()*r_04*h_n, h_n, -this->H.col(2), this->H.col(3));
                    sp2_q34.solve();

                    const std::vector<double> &theta_3 = sp2_q34.get_theta_1();
                    const std::vector<double> &theta_4 = sp2_q34.get_theta_2();
                    
                    for(unsigned j = 0; j < theta_3.size(); ++j)
                    {
                        const double& q3 = theta_3.at(j);
                        const double& q4 = theta_4.at(j);

                        solution.Q.push_back({q1, q2, q3, q4});
                        solution.is_LS_vec.push_back(sp2_q12.solution_is_ls() || sp2_q34.solution_is_ls());
                    }
                }

                break;
            };            
            case KinematicClass::SPHERICAL_WRIST:
            {
                SP1 sp1_q1(this->P.col(1), p_14, this->H.col(0));
                sp1_q1.solve();

                const double& q1 = sp1_q1.get_theta();
                const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();

                SP2 sp2(r_01.transpose()*r_04*this->H.col(3), this->H.col(3), -this->H.col(1), this->H.col(2));
                sp2.solve();

                const std::vector<double> &theta_2 = sp2.get_theta_1();
                const std::vector<double> &theta_3 = sp2.get_theta_2();
                
                const Eigen::Vector3d& h_n = create_normal_vector(this->H.col(3));
                for(unsigned i = 0; i < theta_2.size(); ++i)
                {
                    const double& q2 = theta_2.at(i);
                    const double& q3 = theta_3.at(i);

                    const Eigen::Matrix3d r_21 = Eigen::AngleAxisd(q2, -this->H.col(1).normalized()).toRotationMatrix();
                    const Eigen::Matrix3d r_32 = Eigen::AngleAxisd(q3, -this->H.col(2).normalized()).toRotationMatrix();

                    SP1 sp1_q4(h_n, r_32*r_21*r_01.transpose()*r_04*h_n, this->H.col(3));
                    sp1_q4.solve();

                    solution.Q.push_back({q1, q2, q3, sp1_q4.get_theta()});
                    solution.is_LS_vec.push_back(sp1_q1.solution_is_ls() || sp2.solution_is_ls() || sp1_q4.solution_is_ls());
                }
                break;
            };
            case KinematicClass::REVERSED:
            {
                if(reversed_Robot_ptr)
                {
                    // Use reversed kinematic chain
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
        
        return enforce_solution_consistency(solution, ee_position_orientation);
    }


    std::string General_4R::get_kinematic_family() const
    {
        switch (kinematicClass)
        {
        case KinematicClass::THIRD_FOURTH_INTERSECTING:
            return std::string("4R-THIRD_FOURTH_INTERSECTING");
        case KinematicClass::SECOND_THIRD_INTERSECTING:
            return std::string("4R-SECOND_THIRD_INTERSECTING");
        case KinematicClass::FIRST_SECOND_PARALLEL:
            return std::string("4R-FIRST_SECOND_PARALLEL");
        case KinematicClass::SECOND_THIRD_PARALLEL:
            return std::string("4R-SECOND_THIRD_PARALLEL");
        case KinematicClass::NONE_PARALLEL_NONE_INTERSECTING:
            return std::string("4R-NONE_PARALLEL_NONE_INTERSECTING");
        case KinematicClass::FIRST_TWO_LAST_TWO_INTERSECTING:
            return std::string("4R-FIRST_TWO_LAST_TWO_INTERSECTING");
        case KinematicClass::SPHERICAL_WRIST:
            return std::string("4R-SPHERICAL_WRIST");
        case KinematicClass::REVERSED:
            return reversed_Robot_ptr ? reversed_Robot_ptr->get_kinematic_family() : "4R-Unknown Kinematic Class";
        default:
            return std::string("4R-Unknown Kinematic Class");
        }
    }

};
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
    General_5R::General_5R(const Eigen::Matrix<double, 3, 5> &H, const Eigen::Matrix<double, 3, 6> &P)
        : General_Robot(H, P), H(H), P(P)
    {
        kinematicClass = determine_Kinematic_Class();
    }

    General_5R::KinematicClass General_5R::determine_Kinematic_Class()
    {
        if (this->H.col(0).cross(this->H.col(1)).norm() < ZERO_THRESH && this->H.col(1).cross(this->H.col(2)).norm() < ZERO_THRESH)
        {
            // h1 || h2 || h3
            if (this->H.col(3).cross(this->H.col(4)).norm() < ZERO_THRESH)
            {
                // h4 || h5
                return KinematicClass::FIRST_SECOND_THIRD_PARALLEL_FOURTH_FITH_PARALLEL;
            }
            return KinematicClass::FIRST_SECOND_THIRD_PARALLEL;
        }
        else if (this->H.col(2).cross(this->H.col(3)).norm() < ZERO_THRESH && this->H.col(3).cross(this->H.col(4)).norm() < ZERO_THRESH)
        {
            // h5 || h4 || h3
            const auto &[H_reversed, P_reversed] = reverse_kinematic_chain(this->H, this->P);
            const Eigen::MatrixXd P_reversed_remodeled = EAIK::remodel_kinematics(H_reversed, P_reversed, ZERO_THRESH, ZERO_THRESH);

            this->reversed_Robot_ptr = std::make_unique<General_5R>(H_reversed, P_reversed_remodeled);
            return KinematicClass::REVERSED;
        }

        if (this->H.col(1).cross(this->H.col(2)).norm() < ZERO_THRESH && this->H.col(2).cross(this->H.col(3)).norm() < ZERO_THRESH)
        {
            // h2 || h3 || h4
            return KinematicClass::SECOND_THIRD_FOURTH_PARALLEL;
        }

        if (EAIK::do_axes_intersect(this->H.col(3), this->H.col(4), this->P.col(4), ZERO_THRESH, ZERO_THRESH))
        {
            // h4xh5
            if (EAIK::do_axes_intersect(this->H.col(2), this->H.col(3), this->P.col(3), ZERO_THRESH, ZERO_THRESH))
            {
                const Eigen::Vector3d p03 = P.block<3, 3>(0, 0).rowwise().sum();
                const Eigen::Vector3d intersection = EAIK::calc_intersection(H.col(2), H.col(3), p03, P.col(3), ZERO_THRESH);

                if (EAIK::is_point_on_Axis(H.col(4), p03 + P.col(3) + P.col(4), intersection, ZERO_THRESH))
                {
                    // h3xh4 -> Spherical Wrist at end effector side
                    if (EAIK::do_axes_intersect(this->H.col(0), this->H.col(1), this->P.col(1), ZERO_THRESH, ZERO_THRESH))
                    {
                        // h1xh2
                        return KinematicClass::SPHERICAL_WRIST_FIRST_SECOND_INTERSECTING;
                    }
                    return KinematicClass::SPHERICAL_WRIST;
                }
            }

            if (EAIK::do_axes_intersect(this->H.col(0), this->H.col(1), this->P.col(1), ZERO_THRESH, ZERO_THRESH))
            {
                // h1xh2
                const Eigen::Vector3d intersection = EAIK::calc_intersection(H.col(0), H.col(1), this->P.col(0), P.col(1), ZERO_THRESH);

                if (EAIK::is_point_on_Axis(H.col(2), P.col(0) + P.col(1) + P.col(2), intersection, ZERO_THRESH))
                {
                    // h2xh3 -> Spherical Wrist at robot base
                    const auto &[H_reversed, P_reversed] = reverse_kinematic_chain(this->H, this->P);
                    const Eigen::MatrixXd P_reversed_remodeled = EAIK::remodel_kinematics(H_reversed, P_reversed, ZERO_THRESH, ZERO_THRESH);

                    this->reversed_Robot_ptr = std::make_unique<General_5R>(H_reversed, P_reversed_remodeled);
                    return KinematicClass::REVERSED;
                }
                return KinematicClass::FOURTH_FITH_INTERSECTING_FIRST_SECOND_INTERSECTING;
            }

            if (EAIK::do_axes_intersect(this->H.col(1), this->H.col(2), this->P.col(2), ZERO_THRESH, ZERO_THRESH))
            {
                // h2xh3
                return KinematicClass::FOURTH_FITH_INTERSECTING_SECOND_THIRD_INTERSECTING;
            }

            if (this->H.col(0).cross(this->H.col(1)).norm() < ZERO_THRESH)
            {
                // h1 || h2
                return KinematicClass::FOURTH_FITH_INTERSECTING_FIRST_SECOND_PARALLEL;
            }

            if (this->H.col(1).cross(this->H.col(2)).norm() < ZERO_THRESH)
            {
                // h2 || h3
                return KinematicClass::FOURTH_FITH_INTERSECTING_SECOND_THIRD_PARALLEL;
            }

            return KinematicClass::FOURTH_FITH_INTERSECTING;
        }
        else if (EAIK::do_axes_intersect(this->H.col(0), this->H.col(1), this->P.col(1), ZERO_THRESH, ZERO_THRESH))
        {
            const auto&[H_reversed, P_reversed] = reverse_kinematic_chain(this->H, this->P);
            const Eigen::MatrixXd P_reversed_remodeled = EAIK::remodel_kinematics(H_reversed, P_reversed, ZERO_THRESH, ZERO_THRESH);

            this->reversed_Robot_ptr = std::make_unique<General_5R>(H_reversed, P_reversed_remodeled);
            return KinematicClass::REVERSED;
        }

        if (EAIK::do_axes_intersect(this->H.col(2), this->H.col(3), this->P.col(3), ZERO_THRESH, ZERO_THRESH) && this->H.col(1).cross(this->H.col(2)).norm() < ZERO_THRESH)
        {
            // h3xh4 and h2 || h3
            if (this->H.col(3).cross(this->H.col(4)).norm() < ZERO_THRESH)
            {
                // h4 || h5
                return KinematicClass::THIRD_FOURTH_INTERSECTING_SECOND_THIRD_PARALLEL_FOURTH_FITH_PARALLEL;
            }
            return KinematicClass::THIRD_FOURTH_INTERSECTING_SECOND_THIRD_PARALLEL;
        }
        else if (EAIK::do_axes_intersect(this->H.col(1), this->H.col(2), this->P.col(2), ZERO_THRESH, ZERO_THRESH) && this->H.col(2).cross(this->H.col(3)).norm() < ZERO_THRESH)
        {
            const auto&[H_reversed, P_reversed] = reverse_kinematic_chain(this->H, this->P);
            const Eigen::MatrixXd P_reversed_remodeled = EAIK::remodel_kinematics(H_reversed, P_reversed, ZERO_THRESH, ZERO_THRESH);

            this->reversed_Robot_ptr = std::make_unique<General_5R>(H_reversed, P_reversed_remodeled);
            return KinematicClass::REVERSED;
        }

        return KinematicClass::UNKNOWN;
    }

    IK_Solution General_5R::calculate_IK(const Homogeneous_T &ee_position_orientation) const
    {
        IK_Solution solution;
        const Eigen::Vector3d p_0t = ee_position_orientation.block<3, 1>(0, 3);
        const Eigen::Matrix3d r_05 = ee_position_orientation.block<3, 3>(0, 0);
        const Eigen::Vector3d p_15 = p_0t - this->P.col(0) - r_05 * this->P.col(5);

        switch (this->kinematicClass)
        {
        case KinematicClass::FOURTH_FITH_INTERSECTING:
        {
            SP5 sp5(-this->P.col(1), p_15, this->P.col(2), this->P.col(3), -this->H.col(0), this->H.col(1), this->H.col(2));
            sp5.solve();

            const std::vector<double> &theta_1 = sp5.get_theta_1();
            const std::vector<double> &theta_2 = sp5.get_theta_2();
            const std::vector<double> &theta_3 = sp5.get_theta_3();

            for (unsigned i = 0; i < theta_1.size(); ++i)
            {
                const double &q1 = theta_1.at(i);
                const double &q2 = theta_2.at(i);
                const double &q3 = theta_3.at(i);

                const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();
                const Eigen::Matrix3d r_12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();
                const Eigen::Matrix3d r_23 = Eigen::AngleAxisd(q3, this->H.col(2).normalized()).toRotationMatrix();

                const Eigen::Vector3d hn = (r_05 * this->H.col(4)).cross(r_01 * r_12 * r_23 * this->H.col(3));

                SP2 sp2(r_05.transpose() * hn, r_23.transpose() * r_12.transpose() * r_01.transpose() * hn, this->H.col(4), -this->H.col(3));
                sp2.solve();

                const std::vector<double> &theta_4 = sp2.get_theta_2();
                const std::vector<double> &theta_5 = sp2.get_theta_1();

                for (unsigned k = 0; k < theta_4.size(); ++k)
                {
                    const double &q4 = theta_4.at(k);
                    const double &q5 = theta_5.at(k);

                    solution.Q.push_back({q1, q2, q3, q4, q5});
                    solution.is_LS_vec.push_back(sp5.solution_is_ls() || sp2.solution_is_ls());
                }
            }
            break;
        };

        case KinematicClass::FOURTH_FITH_INTERSECTING_SECOND_THIRD_INTERSECTING:
        {
            SP3 sp3(p_15, this->P.col(1), -this->H.col(0), this->P.col(3).norm());
            sp3.solve();

            for (const auto &q1 : sp3.get_theta())
            {
                const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();
                SP2 sp2_2_3(r_01.transpose() * p_15 - this->P.col(1), this->P.col(3), -this->H.col(1), this->H.col(2));
                sp2_2_3.solve();

                const std::vector<double> &theta_2 = sp2_2_3.get_theta_1();
                const std::vector<double> &theta_3 = sp2_2_3.get_theta_2();

                for (unsigned i = 0; i < theta_2.size(); ++i)
                {
                    const double &q2 = theta_2.at(i);
                    const double &q3 = theta_3.at(i);

                    const Eigen::Matrix3d r_12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();
                    const Eigen::Matrix3d r_23 = Eigen::AngleAxisd(q3, this->H.col(2).normalized()).toRotationMatrix();

                    const Eigen::Vector3d hn = (r_05 * this->H.col(4)).cross(r_01 * r_12 * r_23 * this->H.col(3));

                    SP2 sp2_4_5(r_05.transpose() * hn, r_23.transpose() * r_12.transpose() * r_01.transpose() * hn, this->H.col(4), -this->H.col(3));
                    sp2_4_5.solve();

                    const std::vector<double> &theta_4 = sp2_4_5.get_theta_2();
                    const std::vector<double> &theta_5 = sp2_4_5.get_theta_1();

                    for (unsigned k = 0; k < theta_4.size(); ++k)
                    {
                        const double &q4 = theta_4.at(k);
                        const double &q5 = theta_5.at(k);

                        solution.Q.push_back({q1, q2, q3, q4, q5});
                        solution.is_LS_vec.push_back(sp3.solution_is_ls() || sp2_2_3.solution_is_ls() || sp2_4_5.solution_is_ls());
                    }
                }
            }

            break;
        };

        case KinematicClass::FOURTH_FITH_INTERSECTING_FIRST_SECOND_INTERSECTING:
        {
            SP3 sp3(this->P.col(3), -this->P.col(2), this->H.col(2), p_15.norm());
            sp3.solve();

            for (const auto &q3 : sp3.get_theta())
            {
                const Eigen::Matrix3d r_23 = Eigen::AngleAxisd(q3, this->H.col(2).normalized()).toRotationMatrix();

                SP2 sp2_1_2(p_15, this->P.col(2) + r_23 * this->P.col(3), -this->H.col(0), this->H.col(1));
                sp2_1_2.solve();
                const std::vector<double> &theta_1 = sp2_1_2.get_theta_1();
                const std::vector<double> &theta_2 = sp2_1_2.get_theta_2();

                for (unsigned i = 0; i < theta_1.size(); ++i)
                {
                    const double &q1 = theta_1.at(i);
                    const double &q2 = theta_2.at(i);
                    const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();
                    const Eigen::Matrix3d r_12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();

                    const Eigen::Vector3d hn = (r_05 * this->H.col(4)).cross(r_01 * r_12 * r_23 * this->H.col(3));

                    SP2 sp2_4_5(r_05.transpose() * hn, r_23.transpose() * r_12.transpose() * r_01.transpose() * hn, this->H.col(4), -this->H.col(3));
                    sp2_4_5.solve();

                    const std::vector<double> &theta_4 = sp2_4_5.get_theta_2();
                    const std::vector<double> &theta_5 = sp2_4_5.get_theta_1();

                    for (unsigned k = 0; k < theta_4.size(); ++k)
                    {
                        const double &q4 = theta_4.at(k);
                        const double &q5 = theta_5.at(k);

                        solution.Q.push_back({q1, q2, q3, q4, q5});
                        solution.is_LS_vec.push_back(sp3.solution_is_ls() || sp2_1_2.solution_is_ls() || sp2_4_5.solution_is_ls());
                    }
                }
            }

            break;
        };

        case KinematicClass::FOURTH_FITH_INTERSECTING_FIRST_SECOND_PARALLEL:
        {
            SP4 sp4(this->H.col(0), this->P.col(3), this->H.col(2), this->H.col(0).transpose() * (p_15 - this->P.col(1) - this->P.col(2)));
            sp4.solve();

            for (const auto &q3 : sp4.get_theta())
            {
                const Eigen::Matrix3d r_23 = Eigen::AngleAxisd(q3, this->H.col(2).normalized()).toRotationMatrix();

                SP3 sp3(p_15, this->P.col(1), -this->H.col(0), (this->P.col(2) + r_23 * this->P.col(3)).norm());
                sp3.solve();

                for (const auto &q1 : sp3.get_theta())
                {
                    const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();

                    SP1 sp1(this->P.col(2) + r_23 * this->P.col(3), r_01.transpose() * p_15 - this->P.col(1), this->H.col(1));
                    sp1.solve();

                    const double &q2 = sp1.get_theta();
                    const Eigen::Matrix3d r_12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();

                    const Eigen::Vector3d hn = (r_05 * this->H.col(4)).cross(r_01 * r_12 * r_23 * this->H.col(3));

                    SP2 sp2_4_5(r_05.transpose() * hn, r_23.transpose() * r_12.transpose() * r_01.transpose() * hn, this->H.col(4), -this->H.col(3));
                    sp2_4_5.solve();

                    const std::vector<double> &theta_4 = sp2_4_5.get_theta_2();
                    const std::vector<double> &theta_5 = sp2_4_5.get_theta_1();

                    for (unsigned k = 0; k < theta_4.size(); ++k)
                    {
                        const double &q4 = theta_4.at(k);
                        const double &q5 = theta_5.at(k);

                        solution.Q.push_back({q1, q2, q3, q4, q5});
                        solution.is_LS_vec.push_back(sp3.solution_is_ls() || sp4.solution_is_ls() || sp1.solution_is_ls() || sp2_4_5.solution_is_ls());
                    }
                }
            }
            break;
        };

        case KinematicClass::FOURTH_FITH_INTERSECTING_SECOND_THIRD_PARALLEL:
        {
            SP4 sp4(this->H.col(1), p_15, -this->H.col(0), this->H.col(1).transpose() * (this->P.col(1) + this->P.col(2) + this->P.col(3)));
            sp4.solve();

            for (const auto &q1 : sp4.get_theta())
            {
                const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();
                const Eigen::Vector3d placeholder = r_01.transpose() * p_15 - this->P.col(1);
                SP3 sp3_3(this->P.col(3), -this->P.col(2), this->H.col(2), placeholder.norm());
                sp3_3.solve();

                for (const auto &q3 : sp3_3.get_theta())
                {
                    const Eigen::Matrix3d r_23 = Eigen::AngleAxisd(q3, this->H.col(2).normalized()).toRotationMatrix();
                    SP3 sp3_2(placeholder, this->P.col(2), -this->H.col(1), this->P.col(3).norm());
                    sp3_2.solve();

                    for (const auto &q2 : sp3_2.get_theta())
                    {
                        const Eigen::Matrix3d r_12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();

                        const Eigen::Vector3d hn = (r_05 * this->H.col(4)).cross(r_01 * r_12 * r_23 * this->H.col(3));

                        SP2 sp2_4_5(r_05.transpose() * hn, r_23.transpose() * r_12.transpose() * r_01.transpose() * hn, this->H.col(4), -this->H.col(3));
                        sp2_4_5.solve();

                        const std::vector<double> &theta_4 = sp2_4_5.get_theta_2();
                        const std::vector<double> &theta_5 = sp2_4_5.get_theta_1();

                        for (unsigned k = 0; k < theta_4.size(); ++k)
                        {
                            const double &q4 = theta_4.at(k);
                            const double &q5 = theta_5.at(k);

                            solution.Q.push_back({q1, q2, q3, q4, q5});
                            solution.is_LS_vec.push_back(sp3_2.solution_is_ls() || sp4.solution_is_ls() || sp3_3.solution_is_ls() || sp2_4_5.solution_is_ls());
                        }
                    }
                }
            }
            break;
        };

        case KinematicClass::SPHERICAL_WRIST:
        {
            SP3 sp3(this->P.col(1), p_15, this->H.col(0), this->P.col(2).norm());
            sp3.solve();

            for (const auto &q1 : sp3.get_theta())
            {
                const Eigen::Matrix3d r_10 = Eigen::AngleAxisd(q1, -this->H.col(0).normalized()).toRotationMatrix();
                SP1 sp1_2(this->P.col(2), r_10 * p_15 - this->P.col(1), this->H.col(1));
                sp1_2.solve();

                const double &q2 = sp1_2.get_theta();
                const Eigen::Matrix3d r_21 = Eigen::AngleAxisd(q2, -this->H.col(1).normalized()).toRotationMatrix();

                SP2 sp2(r_21 * r_10 * r_05 * this->H.col(4), this->H.col(4), -this->H.col(2), this->H.col(3));
                sp2.solve();

                const std::vector<double> &theta_3 = sp2.get_theta_1();
                const std::vector<double> &theta_4 = sp2.get_theta_2();

                for (unsigned k = 0; k < theta_3.size(); ++k)
                {
                    const double &q3 = theta_3.at(k);
                    const double &q4 = theta_4.at(k);

                    const Eigen::Matrix3d r_32 = Eigen::AngleAxisd(q3, -this->H.col(2).normalized()).toRotationMatrix();
                    const Eigen::Matrix3d r_43 = Eigen::AngleAxisd(q4, -this->H.col(3).normalized()).toRotationMatrix();

                    const Eigen::Matrix3d r_40 = r_43 * r_32 * r_21 * r_10;
                    const Eigen::Vector3d hn = create_normal_vector(r_40.transpose() * this->H.col(4));
                    SP1 sp1_5(r_43 * r_32 * r_21 * r_10 * hn, r_05.transpose() * hn, -this->H.col(4));
                    sp1_5.solve();

                    solution.Q.push_back({q1, q2, q3, q4, sp1_5.get_theta()});
                    solution.is_LS_vec.push_back(sp3.solution_is_ls() || sp1_2.solution_is_ls() || sp1_5.solution_is_ls() || sp2.solution_is_ls());
                }
            }

            break;
        };

        case KinematicClass::SPHERICAL_WRIST_FIRST_SECOND_INTERSECTING:
        {
            SP2 sp2_1_2(p_15, this->P.col(2), -this->H.col(0), this->H.col(1));
            sp2_1_2.solve();

            const std::vector<double> &theta_1 = sp2_1_2.get_theta_1();
            const std::vector<double> &theta_2 = sp2_1_2.get_theta_2();

            for (unsigned k = 0; k < theta_1.size(); ++k)
            {
                const double &q1 = theta_1.at(k);
                const double &q2 = theta_2.at(k);

                const Eigen::Matrix3d r_10 = Eigen::AngleAxisd(q1, -this->H.col(0).normalized()).toRotationMatrix();
                const Eigen::Matrix3d r_21 = Eigen::AngleAxisd(q2, -this->H.col(1).normalized()).toRotationMatrix();

                SP2 sp2_3_4(r_21 * r_10 * r_05 * this->H.col(4), this->H.col(4), -this->H.col(2), this->H.col(3));
                sp2_3_4.solve();

                const std::vector<double> &theta_3 = sp2_3_4.get_theta_1();
                const std::vector<double> &theta_4 = sp2_3_4.get_theta_2();

                for (unsigned i = 0; i < theta_3.size(); ++i)
                {
                    const double &q3 = theta_3.at(i);
                    const double &q4 = theta_4.at(i);

                    const Eigen::Matrix3d r_32 = Eigen::AngleAxisd(q3, -this->H.col(2).normalized()).toRotationMatrix();
                    const Eigen::Matrix3d r_43 = Eigen::AngleAxisd(q4, -this->H.col(3).normalized()).toRotationMatrix();

                    const Eigen::Matrix3d r_40 = r_43 * r_32 * r_21 * r_10;
                    const Eigen::Vector3d hn = create_normal_vector(r_40.transpose() * this->H.col(4));

                    SP1 sp1(r_40 * hn, r_05.transpose() * hn, -this->H.col(4));
                    sp1.solve();

                    solution.Q.push_back({q1, q2, q3, q4, sp1.get_theta()});
                    solution.is_LS_vec.push_back(sp2_3_4.solution_is_ls() || sp2_1_2.solution_is_ls() || sp1.solution_is_ls());
                }
            }
            break;
        };

        case KinematicClass::THIRD_FOURTH_INTERSECTING_SECOND_THIRD_PARALLEL:
        {
            SP6 sp6(this->H.col(1), this->H.col(1), this->H.col(1), this->H.col(1),
                    -this->H.col(0), this->H.col(3), -this->H.col(0), this->H.col(3),
                    p_15, -this->P.col(4), r_05 * this->H.col(4), -this->H.col(4),
                    this->H.col(1).transpose() * (this->P.col(1) + this->P.col(2)), 0);
            sp6.solve();

            const std::vector<double> &theta_1 = sp6.get_theta_1();
            const std::vector<double> &theta_4 = sp6.get_theta_2();

            for (unsigned i = 0; i < theta_1.size(); ++i)
            {
                const double &q1 = theta_1.at(i);
                const double &q4 = theta_4.at(i);

                const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();
                const Eigen::Matrix3d r_34 = Eigen::AngleAxisd(q4, this->H.col(3).normalized()).toRotationMatrix();

                SP3 sp3(r_34 * this->P.col(4), -this->P.col(2), this->H.col(2), (r_01.transpose() * p_15 - this->P.col(1)).norm());
                sp3.solve();

                for (const auto &q3 : sp3.get_theta())
                {
                    const Eigen::Matrix3d r_23 = Eigen::AngleAxisd(q3, this->H.col(2).normalized()).toRotationMatrix();

                    SP1 sp1_2(this->P.col(2) + r_23 * r_34 * this->P.col(4), r_01.transpose() * p_15 - this->P.col(1), this->H.col(1));
                    sp1_2.solve();
                    const double &q2 = sp1_2.get_theta();

                    const Eigen::Matrix3d r_12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();
                    const Eigen::Vector3d hn = create_normal_vector(this->H.col(4));

                    SP1 sp1_5(hn, r_34.transpose() * r_23.transpose() * r_12.transpose() * r_01.transpose() * r_05 * hn, this->H.col(4));
                    sp1_5.solve();

                    solution.Q.push_back({q1, q2, q3, q4, sp1_5.get_theta()});
                    solution.is_LS_vec.push_back(sp6.solution_is_ls() || sp3.solution_is_ls() || sp1_2.solution_is_ls() || sp1_5.solution_is_ls());
                }
            }
            break;
        };

        case KinematicClass::THIRD_FOURTH_INTERSECTING_SECOND_THIRD_PARALLEL_FOURTH_FITH_PARALLEL:
        {
            SP4 sp4_1(this->H.col(1), r_05 * this->H.col(4), -this->H.col(0), this->H.col(1).transpose() * this->H.col(4));
            sp4_1.solve();

            for (const auto &q1 : sp4_1.get_theta())
            {
                const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();

                SP4 sp4_4(this->H.col(1), this->P.col(4), this->H.col(3), this->H.col(1).transpose() * (r_01.transpose() * p_15 - this->P.col(1) - this->P.col(2)));
                sp4_4.solve();

                for (const auto &q4 : sp4_4.get_theta())
                {
                    const Eigen::Matrix3d r_34 = Eigen::AngleAxisd(q4, this->H.col(3).normalized()).toRotationMatrix();
                    SP3 sp3(r_34 * this->P.col(4), -this->P.col(2), this->H.col(2), (r_01.transpose() * p_15 - this->P.col(1)).norm());
                    sp3.solve();

                    for (const auto &q3 : sp3.get_theta())
                    {
                        const Eigen::Matrix3d r_23 = Eigen::AngleAxisd(q3, this->H.col(2).normalized()).toRotationMatrix();
                        SP1 sp1_2(this->P.col(2) + r_23 * r_34 * this->P.col(4), r_01.transpose() * p_15 - this->P.col(1), this->H.col(1));
                        sp1_2.solve();
                        sp1_2.get_theta();

                        const double &q2 = sp1_2.get_theta();
                        const Eigen::Matrix3d r_12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();

                        const Eigen::Vector3d hn = create_normal_vector(r_05 * this->H.col(4));
                        SP1 sp1_5(r_05.transpose() * hn, r_34.transpose() * r_23.transpose() * r_12.transpose() * r_01.transpose() * hn, this->H.col(4));
                        sp1_5.solve();

                        solution.Q.push_back({q1, q2, q3, q4, sp1_5.get_theta()});
                        solution.is_LS_vec.push_back(sp4_1.solution_is_ls() || sp4_4.solution_is_ls() || sp3.solution_is_ls() || sp1_2.solution_is_ls() || sp1_5.solution_is_ls());
                    }
                }
            }
            break;
        };

        case KinematicClass::FIRST_SECOND_THIRD_PARALLEL:
        {
            SP2 sp2(r_05.transpose() * this->H.col(0), this->H.col(0), this->H.col(4), -this->H.col(3));
            sp2.solve();

            const std::vector<double> &theta_4 = sp2.get_theta_2();
            const std::vector<double> &theta_5 = sp2.get_theta_1();

            for (unsigned i = 0; i < theta_4.size(); ++i)
            {
                const double &q4 = theta_4.at(i);
                const double &q5 = theta_5.at(i);

                const Eigen::Matrix3d r_34 = Eigen::AngleAxisd(q4, this->H.col(3).normalized()).toRotationMatrix();
                const Eigen::Matrix3d r_45 = Eigen::AngleAxisd(q5, this->H.col(4).normalized()).toRotationMatrix();

                const Eigen::Vector3d hn = create_normal_vector(this->H.col(0));
                SP1 sp1_03(hn, r_05*r_45.transpose()*r_34.transpose()*hn, this->H.col(0));
                sp1_03.solve();

                const double &q03 = sp1_03.get_theta();
                const Eigen::Matrix3d r_03 = Eigen::AngleAxisd(q03, this->H.col(0).normalized()).toRotationMatrix();

                SP3 sp3_2(this->P.col(2), -this->P.col(1), this->H.col(1), (p_15 - r_03*(this->P.col(3) + r_34*this->P.col(4))).norm());
                sp3_2.solve();

                for (const auto &q2 : sp3_2.get_theta())
                {
                    const Eigen::Matrix3d r_12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();
                    SP1 sp1_1(this->P.col(1) + r_12*this->P.col(2), p_15 - r_03*(this->P.col(3) + r_34*this->P.col(4)), this->H.col(0));
                    sp1_1.solve();

                    const double &q1 = sp1_1.get_theta();
                    const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();

                    SP1 sp1_3(r_12.transpose()*r_01.transpose()*hn, r_03.transpose()*hn, -this->H.col(2));
                    sp1_3.solve();

                    solution.Q.push_back({q1, q2, sp1_3.get_theta(), q4, q5});
                    solution.is_LS_vec.push_back(sp2.solution_is_ls() || sp1_3.solution_is_ls() || sp1_1.solution_is_ls() || sp3_2.solution_is_ls() || sp1_03.solution_is_ls());
                }
            }

            break;
        };

        case KinematicClass::FIRST_SECOND_THIRD_PARALLEL_FOURTH_FITH_PARALLEL:
        {
            SP4 sp4(this->H.col(0), this->P.col(4), this->H.col(3), this->H.col(0).transpose() * (p_15 - this->P.col(1) - this->P.col(2) - this->P.col(3)));
            sp4.solve();

            for (const auto &q4 : sp4.get_theta())
            {
                const Eigen::Matrix3d r_34 = Eigen::AngleAxisd(q4, this->H.col(3).normalized()).toRotationMatrix();

                SP1 sp1_5(r_05.transpose() * this->H.col(0), r_34.transpose() * this->H.col(0), this->H.col(4));
                sp1_5.solve();

                const double &q5 = sp1_5.get_theta();
                const Eigen::Matrix3d r_45 = Eigen::AngleAxisd(q5, this->H.col(4).normalized()).toRotationMatrix();

                const Eigen::Vector3d hn = create_normal_vector(this->H.col(0));
                SP1 sp1_03(hn, r_05*r_45.transpose()*r_34.transpose()*hn, this->H.col(0));
                sp1_03.solve();

                const double &q03 = sp1_03.get_theta();
                const Eigen::Matrix3d r_03 = Eigen::AngleAxisd(q03, this->H.col(0).normalized()).toRotationMatrix();

                SP3 sp3_2(this->P.col(2), -this->P.col(1), this->H.col(1), (p_15 - r_03*(this->P.col(3) + r_34*this->P.col(4))).norm());
                sp3_2.solve();

                for (const auto &q2 : sp3_2.get_theta())
                {
                    const Eigen::Matrix3d r_12 = Eigen::AngleAxisd(q2, this->H.col(1).normalized()).toRotationMatrix();
                    SP1 sp1_1(this->P.col(1) + r_12*this->P.col(2), p_15 - r_03*(this->P.col(3) + r_34*this->P.col(4)), this->H.col(0));
                    sp1_1.solve();

                    const double &q1 = sp1_1.get_theta();
                    const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();

                    SP1 sp1_3(r_12.transpose()*r_01.transpose()*hn, r_03.transpose()*hn, -this->H.col(2));
                    sp1_3.solve();

                    solution.Q.push_back({q1, q2, sp1_3.get_theta(), q4, q5});
                    solution.is_LS_vec.push_back(sp4.solution_is_ls() || sp1_5.solution_is_ls() || sp1_3.solution_is_ls() || sp1_1.solution_is_ls() || sp3_2.solution_is_ls() || sp1_03.solution_is_ls());
                }
            }
            break;
        };

        case KinematicClass::SECOND_THIRD_FOURTH_PARALLEL:
        {
            SP4 sp4(this->H.col(1), p_15, -this->H.col(0), this->H.col(2).transpose() * (this->P.col(1) + this->P.col(2) + this->P.col(3) + this->P.col(4)));
            sp4.solve();

            for (const auto &q1 : sp4.get_theta())
            {
                const Eigen::Matrix3d r_01 = Eigen::AngleAxisd(q1, this->H.col(0).normalized()).toRotationMatrix();
                SP1 sp1_5(r_05.transpose() * r_01 * this->H.col(1), this->H.col(1), this->H.col(4));
                sp1_5.solve();

                const double &q5 = sp1_5.get_theta();
                const Eigen::Matrix3d r_45 = Eigen::AngleAxisd(q5, this->H.col(4).normalized()).toRotationMatrix();

                const Eigen::Vector3d hn = create_normal_vector(this->H.col(1));
                SP1 sp1_14(hn, r_01.transpose()*r_05*r_45.transpose()*hn, this->H.col(1));
                sp1_14.solve();

                const double &q14 = sp1_14.get_theta();
                const Eigen::Matrix3d r_14 = Eigen::AngleAxisd(q14, this->H.col(1).normalized()).toRotationMatrix();
                const Eigen::Matrix3d r_40 = (r_01*r_14).transpose();

                const Eigen::Vector3d placeholder = r_40*p_15 - r_14.transpose()*this->P.col(1) - this->P.col(4);
                SP3 sp3_3(this->P.col(2), -this->P.col(3), -this->H.col(2), placeholder.norm());
                sp3_3.solve();

                for(const auto& q3 : sp3_3.get_theta())
                {
                    const Eigen::Matrix3d r_32 = Eigen::AngleAxisd(q3, -this->H.col(2).normalized()).toRotationMatrix();
                    SP1 sp1_4(r_32*this->P.col(2) + this->P.col(3), placeholder, -this->H.col(3));
                    sp1_4.solve();
                    const double &q4 = sp1_4.get_theta();
                    const Eigen::Matrix3d r_43 = Eigen::AngleAxisd(q4, -this->H.col(3).normalized()).toRotationMatrix();

                    SP1 sp1_2(hn, r_14*r_43*r_32*hn, this->H.col(1));
                    sp1_2.solve();

                    solution.Q.push_back({q1, sp1_2.get_theta(), q3, q4, q5});
                    solution.is_LS_vec.push_back(sp4.solution_is_ls() || sp1_5.solution_is_ls() || sp3_3.solution_is_ls() || sp1_4.solution_is_ls() || sp1_2.solution_is_ls() || sp1_14.solution_is_ls());
                }
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
            std::cerr << "The choosen manipulator has no known subproblem decomposition! The resulting solutions will be empty." << std::endl;
        }

        return enforce_solution_consistency(solution, ee_position_orientation);
    }

    std::string General_5R::get_kinematic_family() const
    {
        switch (this->kinematicClass)
        {
        case KinematicClass::FOURTH_FITH_INTERSECTING:
            return "5R-FOURTH_FITH_INTERSECTING";
        case KinematicClass::FOURTH_FITH_INTERSECTING_SECOND_THIRD_INTERSECTING:
            return "5R-FOURTH_FITH_INTERSECTING_SECOND_THIRD_INTERSECTING";
        case KinematicClass::FOURTH_FITH_INTERSECTING_FIRST_SECOND_INTERSECTING:
            return "5R-FOURTH_FITH_INTERSECTING_FIRST_SECOND_INTERSECTING";
        case KinematicClass::FOURTH_FITH_INTERSECTING_FIRST_SECOND_PARALLEL:
            return "5R-FOURTH_FITH_INTERSECTING_FIRST_SECOND_PARALLEL";
        case KinematicClass::FOURTH_FITH_INTERSECTING_SECOND_THIRD_PARALLEL:
            return "5R-FOURTH_FITH_INTERSECTING_SECOND_THIRD_PARALLEL";
        case KinematicClass::SPHERICAL_WRIST:
            return "5R-SPHERICAL_WRIST";
        case KinematicClass::SPHERICAL_WRIST_FIRST_SECOND_INTERSECTING:
            return "5R-SPHERICAL_WRIST_FIRST_SECOND_INTERSECTING";
        case KinematicClass::THIRD_FOURTH_INTERSECTING_SECOND_THIRD_PARALLEL:
            return "5R-THIRD_FOURTH_INTERSECTING_SECOND_THIRD_PARALLEL";
        case KinematicClass::THIRD_FOURTH_INTERSECTING_SECOND_THIRD_PARALLEL_FOURTH_FITH_PARALLEL:
            return "5R-THIRD_FOURTH_INTERSECTING_SECOND_THIRD_PARALLEL_FOURTH_FITH_PARALLEL";
        case KinematicClass::FIRST_SECOND_THIRD_PARALLEL:
            return "5R-FIRST_SECOND_THIRD_PARALLEL";
        case KinematicClass::FIRST_SECOND_THIRD_PARALLEL_FOURTH_FITH_PARALLEL:
            return "5R-FIRST_SECOND_THIRD_PARALLEL_FOURTH_FITH_PARALLEL";
        case KinematicClass::SECOND_THIRD_FOURTH_PARALLEL:
            return "5R-SECOND_THIRD_FOURTH_PARALLEL";
        case KinematicClass::REVERSED:
            return reversed_Robot_ptr ? reversed_Robot_ptr->get_kinematic_family() : "5R-Unknown Kinematic Class";
        }
        return "5R-Unknown Kinematic Class";
    }
};
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "kinematic_remodelling.h"
#include "EAIK.h"

namespace EAIK
{
    Robot::Robot(const Eigen::MatrixXd &H, const Eigen::MatrixXd &P, bool is_double_precision)
    {
        if(is_double_precision)
        {
            ZERO_THRESHOLD = 1e-13;
            AXIS_INTERSECT_THRESHOLD = 1e-9;
        }

        Eigen::MatrixXd P_new = remodel_kinematics(H, P, ZERO_THRESHOLD, AXIS_INTERSECT_THRESHOLD);
        original_kinematics = std::make_unique<IKS::General_Robot>(H, P);
        if(H.cols() != 6 || P.cols() != 7)
        {
            throw std::runtime_error("Currently, only 6DOF Robots are solvable with EAIK.");
        }

        // Check if last three axes intersect
        if (P_new.col(P_new.cols()-3).norm() < ZERO_THRESHOLD && P_new.col(P_new.cols()-2).norm() < ZERO_THRESHOLD)
        {   
            spherical_wrist = true;
            bot_kinematics = std::make_unique<IKS::Spherical_Wrist_Robot>(H, P_new);
        }
        else
        {
            bot_kinematics = std::make_unique<IKS::General_Robot>(H, P_new);
        }
    }

    IKS::IK_Solution Robot::calculate_IK(const IKS::Homogeneous_T &ee_position_orientation) const
    {
        return bot_kinematics->calculate_IK(ee_position_orientation);
    }    
    
    IKS::Homogeneous_T Robot::fwdkin(const std::vector<double> &Q) const
    {
        return original_kinematics->fwdkin(Q);
    }

    bool Robot::is_spherical() const
    {
        return spherical_wrist;
    }
} // namespace EAIK

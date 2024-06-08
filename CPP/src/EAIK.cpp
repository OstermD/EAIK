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
        if(P.cols() != H.cols() + 1)
        {
            throw std::runtime_error("Wrong input dimensions for H and P. Note that #P = #H+1.");
        }

        switch (H.cols())
        {
        case 6:
            // Check if last three axes intersect
            if (P_new.col(P_new.cols()-3).norm() < ZERO_THRESHOLD && P_new.col(P_new.cols()-2).norm() < ZERO_THRESHOLD)
            {   
                spherical_wrist = true;
                bot_kinematics = std::make_unique<IKS::Spherical_Wrist_Robot>(H, P_new);
                original_kinematics = std::make_unique<IKS::Spherical_Wrist_Robot>(H, P);
            }
            else
            {
                bot_kinematics = std::make_unique<IKS::General_6R>(H, P_new);
                original_kinematics = std::make_unique<IKS::General_6R>(H, P);
            }
            break;
        
        case 3:
                bot_kinematics = std::make_unique<IKS::General_3R>(H, P_new);
                original_kinematics = std::make_unique<IKS::General_3R>(H, P);  
            break;
        default:
            throw std::runtime_error("Currently, only 6R and 3R Robots are solvable with EAIK.");
            break;
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

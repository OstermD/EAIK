#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <math.h>

#include "sp.h"
#include "IKS.h"
#include "utils/kinematic_utils.h"

namespace IKS
{
    General_5R::General_5R(const Eigen::Matrix<double, 3, 5> &H, const Eigen::Matrix<double, 3, 6> &P)
        : General_Robot(H,P), H(H), P(P)
    {
    }

    IK_Solution General_5R::calculate_IK(const Homogeneous_T &ee_position_orientation) const
    {
        IK_Solution solution;
        IK_Solution solution_t_12;
        const Eigen::Vector3d p_0t = ee_position_orientation.block<3, 1>(0, 3);
        const Eigen::Matrix3d r_03 = ee_position_orientation.block<3, 3>(0, 0);
        const Eigen::Vector3d p_13 = p_0t - this->P.col(0) - r_03*this->P.col(3);        

        
        return solution;
    }
};
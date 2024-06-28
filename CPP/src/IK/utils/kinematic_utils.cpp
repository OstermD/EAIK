#include <eigen3/Eigen/Dense>
#include <math.h>

#include "kinematic_utils.h"

namespace IKS
{
    Eigen::Vector3d create_normal_vector(const Eigen::Vector3d& v, const double numerical_threshold)
    {
        // Create some (numerically stable) normal vector to v 
        Eigen::Vector3d hn = v.cross(Eigen::Vector3d(1,0,0)); 

        // As v might be nearly equal to (1,0,0)
        if(hn.isZero(numerical_threshold))
        {
            hn = v.cross(Eigen::Vector3d(0,1,0));
        }

        return hn.normalized();
    }

    Eigen::Matrix<double, 4, 4> inverse_homogeneous_T(const Eigen::Matrix<double, 4, 4> &trafo)
    {
        Eigen::Matrix<double, 4, 4> inverse = Eigen::Matrix<double, 4, 4>::Identity();
        inverse.block<3, 1>(0, 3) = -trafo.block<3, 3>(0, 0).transpose()*trafo.block<3, 1>(0, 3);
        inverse.block<3, 3>(0, 0) = trafo.block<3, 3>(0, 0).transpose();

        return inverse;
    }
} // namespace IKS

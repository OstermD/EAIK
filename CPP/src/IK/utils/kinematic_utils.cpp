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

    // Reverse Kinematic chain: Be careful, angle parametrization reverses too!
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> reverse_kinematic_chain(const Eigen::MatrixXd &H, const Eigen::MatrixXd &P)
    {
        return {(-H).rowwise().reverse(), (-P).rowwise().reverse()};
    }

    void reverse_vector_second_dimension(std::vector<std::vector<double>> &vector)
    {
        for(auto& inner_v : vector)
        {
            std::reverse(inner_v.begin(), inner_v.end());
        }
    }
} // namespace IKS

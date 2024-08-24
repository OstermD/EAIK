#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>
#include "kinematic_utils.h"

namespace IKS
{
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::Matrix3d> dh_to_H_P(const Eigen::VectorXd& dh_alpha, const Eigen::VectorXd& dh_a, const Eigen::VectorXd& dh_d)
    {
        Eigen::MatrixXd H (3, dh_a.size()); 
        Eigen::MatrixXd P (3, dh_a.size()+1); 
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        P.col(0) = Eigen::Vector3d::Zero();
        for (unsigned i = 0; i < dh_d.size(); ++i)
        {
            Eigen::Matrix3d R_loc;
            R_loc << 1, 0, 0,
                     0, std::cos(dh_alpha[i]), -std::sin(dh_alpha[i]),
                     0, std::sin(dh_alpha[i]), std::cos(dh_alpha[i]);
            H.col(i) = R*Eigen::Vector3d(0,0,1);
            P.col(i+1) = R*Eigen::Vector3d(dh_a[i], 0, dh_d[i]);
            R = R.eval()*R_loc;
        }

        return {H, P, R};
    }

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

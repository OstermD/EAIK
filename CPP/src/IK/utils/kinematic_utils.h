#include <eigen3/Eigen/Dense>
#include "IKS.h"

namespace IKS
{
    Eigen::Vector3d create_normal_vector(const Eigen::Vector3d& v, const double numerical_threshold=1e-10);
    Eigen::Matrix<double, 4, 4> inverse_homogeneous_T(const Eigen::Matrix<double, 4, 4> &trafo);

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> reverse_kinematic_chain(const Eigen::MatrixXd &H, const Eigen::MatrixXd &P);
    void reverse_vector_second_dimension(std::vector<std::vector<double>> &vector);

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::Matrix3d> dh_to_H_P(const Eigen::VectorXd& dh_alpha, const Eigen::VectorXd& dh_a, const Eigen::VectorXd& dh_d);
} // namespace IKS


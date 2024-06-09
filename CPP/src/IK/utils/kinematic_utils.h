#include <eigen3/Eigen/Dense>

namespace IKS
{
    Eigen::Vector3d create_normal_vector(const Eigen::Vector3d& v, const double numerical_threshold=1e-10);
} // namespace IKS


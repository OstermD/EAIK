#ifndef EAIK_IMPL
#define EAIK_IMPL

#include <eigen3/Eigen/Dense>
#include <memory>

#include "IKS.h"

namespace EAIK
{
    class Robot
    {
    public:
        Robot(const Eigen::MatrixXd &H, const Eigen::MatrixXd &P, bool is_double_precision=true);
        IKS::IK_Solution calculate_IK(const IKS::Homogeneous_T &ee_position_orientation) const;
        std::vector<IKS::IK_Solution> calculate_IK_batched(std::vector<IKS::Homogeneous_T> EE_pose_batch, const unsigned worker_threads) const;

        IKS::IK_Eigen_Solution calculate_Eigen_IK(const IKS::Homogeneous_T &ee_position_orientation) const;
        //std::vector<IKS::IK_Solution> calculate_Eigen_IK_batched(std::vector<IKS::Homogeneous_T> EE_pose_batch, const unsigned worker_threads) const;

        IKS::Homogeneous_T fwdkin(const std::vector<double> &Q) const;
        IKS::Homogeneous_T fwdkin_Eigen(const Eigen::VectorXd &Q) const;

        bool is_spherical() const;
    private:
        std::unique_ptr<IKS::General_Robot> bot_kinematics;
        std::unique_ptr<IKS::General_Robot> original_kinematics; // Forward Kinematics calculated by original kinematics 
        bool spherical_wrist{false};

        // single precision default thresholds
        double ZERO_THRESHOLD = 1e-7; 
        double AXIS_INTERSECT_THRESHOLD = 1e-6;
    };
} // namespace EAIK
#endif
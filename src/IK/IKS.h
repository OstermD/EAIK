#ifndef SPHERICAL_IK_H
#define SPHERICAL_IK_H

#include <vector>
#include <eigen3/Eigen/Dense>

namespace IKS
{
    using Homogeneous_T = Eigen::Matrix<double, 4, 4>;

    struct IK_Solution
    {
        std::vector<std::vector<double>> Q;
        std::vector<bool> is_LS_vec;
    };

    Homogeneous_T fwd_kinematics_ndof(const Eigen::MatrixXd &H, const Eigen::MatrixXd &P, const std::vector<double> &Q);

    class General_Robot
    {
        // General 6DOF robot kinematics
    public:
        General_Robot(const Eigen::Matrix<double, 3, 6> &H, const Eigen::Matrix<double, 3, 7> &P);
        virtual IK_Solution calculate_IK(const Homogeneous_T &ee_position_orientation) const;
        virtual Homogeneous_T fwdkin(const std::vector<double> &Q) const final;

    protected:
        Eigen::Matrix<double, 3, 6> H;
        Eigen::Matrix<double, 3, 7> P;
    };

    class Spherical_Wrist_Robot : public General_Robot
    {
        // 6DOF Robot kinematics with spherical wrist (3 consecutive intersecting axes at the endeffector)
    public:
        Spherical_Wrist_Robot(const Eigen::Matrix<double, 3, 6> &H, const Eigen::Matrix<double, 3, 7> &P);
        IK_Solution calculate_IK(const Homogeneous_T &ee_position_orientation) const override;
    };
}

#endif
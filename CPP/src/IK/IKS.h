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
        unsigned num_solutions() const { return Q.size(); }
    };

    struct IK_Eigen_Solution
    {
        Eigen::MatrixXd Q;
        Eigen::Array<bool, Eigen::Dynamic, 1> is_LS_vec;
        unsigned num_solutions() const { return Q.rows(); }
    };

    class General_Robot
    {
        // General Robot kinematics
    public:
        General_Robot(const Eigen::MatrixXd &H, const Eigen::MatrixXd &P);
        virtual IK_Solution calculate_IK(const Homogeneous_T &ee_position_orientation) const;
        virtual Homogeneous_T fwdkin(const std::vector<double> &Q) const final;

        virtual bool has_known_decomposition() const;

        const Eigen::MatrixXd& get_H() const;
        const Eigen::MatrixXd& get_P() const;
    private:
        Eigen::MatrixXd H;
        Eigen::MatrixXd P;
    };

    class General_3R : public General_Robot
    {
        // General 3R Manipulator
    public:
        General_3R(const Eigen::Matrix<double, 3, 3> &H, const Eigen::Matrix<double, 3, 4> &P);
        IK_Solution calculate_IK(const Homogeneous_T &ee_position_orientation) const override;

        bool has_known_decomposition() const override {return true;}

        const Eigen::Matrix<double, 3, 3>& get_H() const;
        const Eigen::Matrix<double, 3, 4>& get_P() const;
    private:
        Eigen::Matrix<double, 3, 3> H;
        Eigen::Matrix<double, 3, 4> P;
    };

    class General_6R : public General_Robot
    {
        // 6DOF Robot kinematics with spherical wrist (3 consecutive intersecting axes at the endeffector)
    public:
        General_6R(const Eigen::Matrix<double, 3, 6> &H, const Eigen::Matrix<double, 3, 7> &P);
        IK_Solution calculate_IK(const Homogeneous_T &ee_position_orientation) const override;

        bool has_known_decomposition() const override;

        const Eigen::Matrix<double, 3, 6>& get_H() const;
        const Eigen::Matrix<double, 3, 7>& get_P() const;
    private:
        Eigen::Matrix<double, 3, 6> H;
        Eigen::Matrix<double, 3, 7> P;
    };

    class Spherical_Wrist_Robot : public General_Robot
    {
        // 6DOF Robot kinematics with spherical wrist (3 consecutive intersecting axes at the endeffector)
    public:
        Spherical_Wrist_Robot(const Eigen::Matrix<double, 3, 6> &H, const Eigen::Matrix<double, 3, 7> &P, const bool use_inverted_chain=false);
        IK_Solution calculate_IK(const Homogeneous_T &ee_position_orientation) const override;

        bool has_known_decomposition() const override {return true;}

    private:
        Eigen::Matrix<double, 3, 6> H;
        Eigen::Matrix<double, 3, 7> P;

        bool use_inverted_chain;
    };
}

#endif
#ifndef SPHERICAL_IK_H
#define SPHERICAL_IK_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <memory>

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

        bool has_known_decomposition() const override { return true; }

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

    private:
        enum KinematicClass
        {
            THREE_INNER_PARALLEL = 0,
            THREE_PARALLEL_TWO_INTERSECTING = 1,
            SPHERICAL_FIRST_TWO_PARALLEL = 2,
            SPHERICAL_SECOND_TWO_PARALLEL = 3,
            SPHERICAL_FIRST_TWO_INTERSECTING = 4,
            SPHERICAL_SECOND_TWO_INTERSECTING = 5,
            SPHERICAL_NO_PARALLEL_NO_INTERSECTING = 6,
            REVERSED = 7,
            UNKNOWN
        };

        KinematicClass determine_Kinematic_Class();
        IK_Solution calculate_Spherical_Wrist_Orientation_Kinematics(const std::vector<std::vector<double>>& position_solutions, const std::vector<bool>& position_solution_is_LS, const Eigen::Matrix3d& r_06) const;

        Eigen::Matrix<double, 3, 6> H;
        Eigen::Matrix<double, 3, 7> P;

        KinematicClass kinematicClass{KinematicClass::UNKNOWN};

        std::unique_ptr<General_6R> reversed_Robot_ptr;   // If kinematic class demands kinematic inversion, this robot will be used
    };
}

#endif
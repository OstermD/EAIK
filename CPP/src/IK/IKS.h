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
        virtual IK_Solution calculate_position_IK(const Eigen::Vector3d &ee_position) const;
        virtual IK_Solution calculate_orientation_IK(const Eigen::Matrix3d &ee_orientation) const;

        virtual Homogeneous_T fwdkin(const std::vector<double> &Q) const final;

        virtual bool has_known_decomposition() const;
        virtual bool is_spherical() const;

        virtual Eigen::MatrixXd get_H() const final { return H; }
        virtual Eigen::MatrixXd get_P() const final { return P; }

        virtual std::string get_kinematic_family() const { return std::string("Unknown"); }

    private:
        Eigen::MatrixXd H;
        Eigen::MatrixXd P;
    };

    class General_1R : public General_Robot
    {
        // General 1R Manipulator
    public:
        General_1R(const Eigen::Matrix<double, 3, 1> &H, const Eigen::Matrix<double, 3, 2> &P);
        IK_Solution calculate_IK(const Homogeneous_T &ee_position_orientation) const override;
        IK_Solution calculate_position_IK(const Eigen::Vector3d &ee_position) const;
        IK_Solution calculate_orientation_IK(const Eigen::Matrix3d &ee_orientation) const;

        bool has_known_decomposition() const override { return true; }

        std::string get_kinematic_family() const override { return std::string("1R"); }

    private:
        Eigen::Matrix<double, 3, 1> H;
        Eigen::Matrix<double, 3, 2> P;
    };

    class General_2R : public General_Robot
    {
        // General 2R Manipulator
    public:
        General_2R(const Eigen::Matrix<double, 3, 2> &H, const Eigen::Matrix<double, 3, 3> &P);
        IK_Solution calculate_IK(const Homogeneous_T &ee_position_orientation) const override;
        IK_Solution calculate_position_IK(const Eigen::Vector3d &ee_position) const;
        IK_Solution calculate_orientation_IK(const Eigen::Matrix3d &ee_orientation) const;

        bool has_known_decomposition() const override { return true; }

        std::string get_kinematic_family() const override { return std::string("2R"); }

    private:
        Eigen::Matrix<double, 3, 2> H;
        Eigen::Matrix<double, 3, 3> P;
    };

    class General_3R : public General_Robot
    {
        // General 3R Manipulator
    public:
        General_3R(const Eigen::Matrix<double, 3, 3> &H, const Eigen::Matrix<double, 3, 4> &P);
        IK_Solution calculate_IK(const Homogeneous_T &ee_position_orientation) const override;
        IK_Solution calculate_position_IK(const Eigen::Vector3d &ee_position) const;
        IK_Solution calculate_orientation_IK(const Eigen::Matrix3d &ee_orientation) const;

        bool has_known_decomposition() const override { return true; }

        std::string get_kinematic_family() const override { return std::string("3R"); }

    private:
        Eigen::Matrix<double, 3, 3> H;
        Eigen::Matrix<double, 3, 4> P;
    };

    class General_4R : public General_Robot
    {
        // General 4R Manipulator
    public:
        General_4R(const Eigen::Matrix<double, 3, 4> &H, const Eigen::Matrix<double, 3, 5> &P);
        IK_Solution calculate_IK(const Homogeneous_T &ee_position_orientation) const override;

        bool has_known_decomposition() const override { return true; }

        std::string get_kinematic_family() const override { return std::string("4R"); }

    private:
        Eigen::Matrix<double, 3, 4> H;
        Eigen::Matrix<double, 3, 5> P;
    };

    class General_5R : public General_Robot
    {
        // General 5R Manipulator
    public:
        General_5R(const Eigen::Matrix<double, 3, 5> &H, const Eigen::Matrix<double, 3, 6> &P);
        IK_Solution calculate_IK(const Homogeneous_T &ee_position_orientation) const override;
        
        bool has_known_decomposition() const override { return true; }

        std::string get_kinematic_family() const override { return std::string("5R"); }

    private:
        Eigen::Matrix<double, 3, 5> H;
        Eigen::Matrix<double, 3, 6> P;
    };

    class General_6R : public General_Robot
    {
        // General 6R Manipulator
    public:
        General_6R(const Eigen::Matrix<double, 3, 6> &H, const Eigen::Matrix<double, 3, 7> &P);
        IK_Solution calculate_IK(const Homogeneous_T &ee_position_orientation) const override;

        bool has_known_decomposition() const override;
        bool is_spherical() const override;

        std::string get_kinematic_family() const override;

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
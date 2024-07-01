#include <iostream>
#include <eigen3/Eigen/Dense>
#include <thread>
#include <mutex>

#include "kinematic_remodelling.h"
#include "kinematic_utils.h"
#include "EAIK.h"

namespace EAIK
{
    Robot::Robot(const Eigen::MatrixXd &H, const Eigen::MatrixXd &P, const Eigen::Matrix<double, 3, 3> R6T, const std::vector<std::pair<int, double>>& fixed_axes, bool is_double_precision) : R6T(R6T)
    {
        if(is_double_precision)
        {
            ZERO_THRESHOLD = 1e-13;
            AXIS_INTERSECT_THRESHOLD = 1e-9;
        }

        if(P.cols() != H.cols() + 1)
        {
            throw std::runtime_error("Wrong input dimensions for H and P. Note that #P = #H+1.");
        }

        Eigen::MatrixXd P_remodelled;
        Eigen::MatrixXd H_remodelled;

        // Insert fixed axes (e.g., to cope with >6DOF manipulators) into the kinematic chain
        if(fixed_axes.size()>0)
        {
            this->fixed_axes = fixed_axes;
            std::sort(this->fixed_axes.begin(), this->fixed_axes.end(), [](const std::pair<int, double>& a, const std::pair<int, double>& b) {return a.first < b.first;});

            const auto&[H_part, P_part, R6T_part] = partial_joint_parametrization(H, P, fixed_axes, R6T);
            P_remodelled = remodel_kinematics(H_part, P_part, ZERO_THRESHOLD, AXIS_INTERSECT_THRESHOLD);
            H_remodelled = H_part;
            this->R6T_partial = R6T_part;
        }
        else
        {
            P_remodelled = remodel_kinematics(H, P, ZERO_THRESHOLD, AXIS_INTERSECT_THRESHOLD);
            H_remodelled = H;
            this->R6T_partial = R6T;
        }

        switch (H_remodelled.cols())
        {
        case 6:
            // Check if last three axes intersect
            if (P_remodelled.col(P_remodelled.cols()-3).norm() < ZERO_THRESHOLD && P_remodelled.col(P_remodelled.cols()-2).norm() < ZERO_THRESHOLD)
            {   
                spherical_wrist = true;
                bot_kinematics = std::make_unique<IKS::Spherical_Wrist_Robot>(H_remodelled, P_remodelled);
                original_kinematics = std::make_unique<IKS::General_Robot>(H, P);
            }
            else if(P_remodelled.col(1).norm() < ZERO_THRESHOLD && P_remodelled.col(2).norm() < ZERO_THRESHOLD)
            {
                // Spherical wrist is located at the base of the robot
                spherical_wrist = true;
                const auto&[H_reversed, P_reversed] = IKS::reverse_kinematic_chain(H_remodelled, P_remodelled);
                bot_kinematics = std::make_unique<IKS::Spherical_Wrist_Robot>(H_reversed, P_reversed, true);
                original_kinematics = std::make_unique<IKS::General_Robot>(H, P);
            }
            else 
            {
                bot_kinematics = std::make_unique<IKS::General_6R>(H_remodelled, P_remodelled);
                original_kinematics = std::make_unique<IKS::General_Robot>(H, P);
            }
            break;
        
        case 3:
                bot_kinematics = std::make_unique<IKS::General_3R>(H_remodelled, P_remodelled);
                original_kinematics = std::make_unique<IKS::General_Robot>(H, P);  
            break;
        default:
            throw std::runtime_error("Currently, only 6R and 3R Robots are solvable with EAIK.");
            break;
        }
    }

    IKS::IK_Solution Robot::calculate_IK(const IKS::Homogeneous_T &ee_position_orientation) const
    {
        IKS::Homogeneous_T ee_06_rot = ee_position_orientation;
        ee_06_rot.block<3,3>(0,0) *= R6T_partial.transpose();
        IKS::IK_Solution solution = bot_kinematics->calculate_IK(ee_06_rot);

        // TODO: Find alternative to this costly insert operation
        for(const auto&[joint_index, value] : fixed_axes)
        {
            for(auto& Qs : solution.Q)
            {
                Qs.insert(Qs.begin()+joint_index, value);
            }
        }

        return solution;
    }    
    
    std::vector<IKS::IK_Solution> Robot::calculate_IK_batched(std::vector<IKS::Homogeneous_T> EE_pose_batch, const unsigned worker_threads) const
    {
        std::vector<IKS::IK_Solution> solutions(EE_pose_batch.size());
        std::vector<std::thread> thread_pool;
        thread_pool.reserve(worker_threads);

        std::mutex pose_mutex;

        auto kernel = 
        [this, &pose_mutex, &solutions, &EE_pose_batch]{
            pose_mutex.lock();
            int solution_index = EE_pose_batch.size()-1;

            while(solution_index >= 0)
            {
                const IKS::Homogeneous_T& pose = EE_pose_batch.back();
                EE_pose_batch.pop_back();  
                pose_mutex.unlock(); 

                solutions.at(solution_index) = calculate_IK(pose);

                pose_mutex.lock();
                solution_index = EE_pose_batch.size()-1;
            }
            pose_mutex.unlock(); 
        };

        for(unsigned i = 0; i < worker_threads; i++)
        {
            thread_pool.push_back(std::thread(kernel));
        }

        for(unsigned i = 0; i < worker_threads; i++)
        {
            thread_pool.at(i).join();
        }

        return solutions;
    }

    // Eigen matrices as solutions for the pybindings
    IKS::IK_Eigen_Solution Robot::calculate_Eigen_IK(const IKS::Homogeneous_T &ee_position_orientation) const
    {
        IKS::IK_Solution vector_solution = bot_kinematics->calculate_IK(ee_position_orientation);
        IKS::IK_Eigen_Solution eigen_solution;

        if(vector_solution.Q.size() > 0)
        {
            eigen_solution.is_LS_vec.resize(vector_solution.is_LS_vec.size());

            // TODO: Find a way to avoid slow loop in future releases
            for(unsigned i = 0; i < vector_solution.is_LS_vec.size(); ++i)
            {
                eigen_solution.is_LS_vec(i) = vector_solution.is_LS_vec.at(i);
            }

            eigen_solution.Q = Eigen::MatrixXd(vector_solution.Q.size(), vector_solution.Q.at(0).size());
            for (unsigned i = 0; i < vector_solution.Q.size(); i++)
            {
                eigen_solution.Q.row(i) = Eigen::VectorXd::Map(vector_solution.Q.at(i).data(),vector_solution.Q.at(0).size());
            }
        }
        return eigen_solution;                
    }

    std::vector<IKS::IK_Eigen_Solution> Robot::calculate_Eigen_IK_batched(std::vector<IKS::Homogeneous_T> EE_pose_batch, const unsigned worker_threads) const
    {
        std::vector<IKS::IK_Eigen_Solution> solutions(EE_pose_batch.size());
        std::vector<std::thread> thread_pool;
        thread_pool.reserve(worker_threads);

        std::mutex pose_mutex;

        auto kernel = 
        [this, &pose_mutex, &solutions, &EE_pose_batch]{
            pose_mutex.lock();
            int solution_index = EE_pose_batch.size()-1;

            while(solution_index >= 0)
            {
                const IKS::Homogeneous_T& pose = EE_pose_batch.back();
                EE_pose_batch.pop_back();  
                pose_mutex.unlock(); 

                solutions.at(solution_index) = calculate_Eigen_IK(pose);

                pose_mutex.lock();
                solution_index = EE_pose_batch.size()-1;
            }
            pose_mutex.unlock(); 
        };

        for(unsigned i = 0; i < worker_threads; i++)
        {
            thread_pool.push_back(std::thread(kernel));
        }

        for(unsigned i = 0; i < worker_threads; i++)
        {
            thread_pool.at(i).join();
        }

        return solutions;
    }    

    IKS::Homogeneous_T Robot::fwdkin(const std::vector<double> &Q) const
    {
        IKS::Homogeneous_T sol_r06 = original_kinematics->fwdkin(Q);
        sol_r06.block<3,3>(0,0) *= R6T;
        return sol_r06;
    }

    // Function for use with pybindings and the corresponding numpy arrays
    IKS::Homogeneous_T Robot::fwdkin_Eigen(const Eigen::VectorXd &Q) const
    {
        std::vector<double> Q_stdVector(Q.data(), Q.data() + Q.size());
        return original_kinematics->fwdkin(Q_stdVector);
    }
    
    bool Robot::is_spherical() const
    {
        return spherical_wrist;
    }
} // namespace EAIK

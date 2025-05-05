#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <math.h>

#include "kinematic_remodeling.h"


namespace EAIK
{
    bool do_axes_intersect(const Eigen::Vector3d& h1, const Eigen::Vector3d& h2, const Eigen::Vector3d& p12, const double ZERO_THRESHOLD, const double AXIS_INTERSECT_THRESHOLD)
    {
        return fabs((h1.cross(h2)).transpose()*p12) < AXIS_INTERSECT_THRESHOLD && (h1.cross(h2)).norm() > ZERO_THRESHOLD;
    }

    bool is_point_on_Axis(const Eigen::Vector3d& h, const Eigen::Vector3d& p0h, const Eigen::Vector3d& p, const double AXIS_INTERSECT_THRESHOLD)
    {
        const Eigen::Vector3d p_h = p - p0h;
        const Eigen::Vector3d proj_p = p_h.dot(h)*h;

        return (proj_p-p_h).norm() < AXIS_INTERSECT_THRESHOLD;
    }

    Eigen::Vector3d calc_intersection(const Eigen::Vector3d& hj, const Eigen::Vector3d& hk, const Eigen::Vector3d& p0j, const Eigen::Vector3d& pkj, const double ZERO_THRESHOLD)
    {
        const Eigen::Vector3d hj_cross_hk = hj.cross(hk);

        if(hj_cross_hk.squaredNorm() < ZERO_THRESHOLD)
        {
            throw std::runtime_error("Intersection point can't be calculated for two parallel axes!");
        }

        Eigen::Matrix3d A;
        A << pkj, hk, hj_cross_hk;
        double lambda_1 = A.determinant()/hj_cross_hk.squaredNorm();
        
        return p0j + hj*lambda_1;
        /*
        To get an error-measure calculate the second lambda and calc the eucledian difference between the two intersection points
        A.col(1) = hj;
        double lambda_2 = A.determinant()/hj_cross_hk.squaredNorm();
        */
    }

    Eigen::MatrixXd remodel_kinematics(const Eigen::MatrixXd &H, const Eigen::MatrixXd &P, const double ZERO_THRESHOLD, const double AXIS_INTERSECT_THRESHOLD)
    {
        Eigen::MatrixXd P_new = P;
        Eigen::Vector3d p0_i_plus1(0,0,0);

        if(P.cols() == 7)
        {
            // Remodel Robot "Base"
            for(unsigned i = 0; i < H.cols()-4; i++)
            {
                p0_i_plus1 += P_new.col(i);
                if (do_axes_intersect(H.col(i), H.col(i+1), P.col(i+1), ZERO_THRESHOLD, AXIS_INTERSECT_THRESHOLD))
                {
                    Eigen::Vector3d intersection = calc_intersection(H.col(i), H.col(i+1), p0_i_plus1, P.col(i+1), ZERO_THRESHOLD);
                    P_new.col(i+1) = Eigen::Vector3d(0,0,0);
                
                    unsigned j = i+2;
                    Eigen::Vector3d p0j = p0_i_plus1+P.col(j-1)+P.col(j);

                    while(j < H.cols()-3 && is_point_on_Axis(H.col(j), p0j, intersection, AXIS_INTERSECT_THRESHOLD))
                    {
                        P_new.col(j) = Eigen::Vector3d(0,0,0);
                        j++;
                        p0j += P.col(j);
                    }
                    P_new.col(i) = intersection - (p0_i_plus1-P_new.col(i));
                    P_new.col(j) = p0j - intersection;
                    i = j-1;
                    p0_i_plus1 = p0j - P.col(j);
                }
            }

            p0_i_plus1+=P.col(H.cols()-4);

            // Remodel Robot "Wrist"
            for(unsigned i = H.cols()-3; i < H.cols()-1; i++)
            {
                p0_i_plus1 += P_new.col(i);
                if (do_axes_intersect(H.col(i), H.col(i+1), P.col(i+1), ZERO_THRESHOLD, AXIS_INTERSECT_THRESHOLD))
                {
                    Eigen::Vector3d intersection = calc_intersection(H.col(i), H.col(i+1), p0_i_plus1, P_new.col(i+1), AXIS_INTERSECT_THRESHOLD);
                
                    unsigned j = i+2;
                    Eigen::Vector3d p0_j_plus1 = p0_i_plus1+P_new.col(i+1)+P_new.col(j);

                    P_new.col(i+1) = Eigen::Vector3d(0,0,0);

                    while(j < H.cols() && is_point_on_Axis(H.col(j), p0_j_plus1, intersection, AXIS_INTERSECT_THRESHOLD))
                    {
                        P_new.col(j) = Eigen::Vector3d(0,0,0);
                        j++;
                        p0_j_plus1 += P.col(j);
                    }
                    P_new.col(i) = intersection - (p0_i_plus1-P_new.col(i));
                    P_new.col(j) = p0_j_plus1 - intersection;
                    i = j-2;
                    p0_i_plus1 = p0_j_plus1 - P.col(j)-P.col(j-1);
                }
            }
        }
        else
        {
            for(unsigned i = 0; i < H.cols()-1; i++)
            {
                p0_i_plus1 += P_new.col(i);
                if (do_axes_intersect(H.col(i), H.col(i+1), P.col(i+1), ZERO_THRESHOLD, AXIS_INTERSECT_THRESHOLD))
                {
                    Eigen::Vector3d intersection = calc_intersection(H.col(i), H.col(i+1), p0_i_plus1, P_new.col(i+1), AXIS_INTERSECT_THRESHOLD);
                
                    unsigned j = i+2;
                    Eigen::Vector3d p0_j_plus1 = p0_i_plus1+P_new.col(i+1)+P_new.col(j);

                    P_new.col(i+1) = Eigen::Vector3d(0,0,0);

                    while(j < H.cols() && is_point_on_Axis(H.col(j), p0_j_plus1, intersection, AXIS_INTERSECT_THRESHOLD))
                    {
                        P_new.col(j) = Eigen::Vector3d(0,0,0);
                        j++;
                        p0_j_plus1 += P.col(j);
                    }
                    P_new.col(i) = intersection - (p0_i_plus1-P_new.col(i));
                    P_new.col(j) = p0_j_plus1 - intersection;
                    i = j-2;
                    p0_i_plus1 = p0_j_plus1 - P.col(j)-P.col(j-1);
                }
            }
        }

        return P_new;
    }

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::Matrix<double, 3, 3>> partial_joint_parametrization(const Eigen::MatrixXd &H, const Eigen::MatrixXd &P, std::vector<std::pair<int, double>> fixed_axes, const Eigen::Matrix<double, 3, 3>& R6T)
    {
        // Sort fixed axes parametrization to start with last axis first
        std::sort(fixed_axes.begin(), fixed_axes.end(), [](const std::pair<int, double>& a, const std::pair<int, double>& b) {return a.first > b.first;});
        Eigen::MatrixXd H_new = H;
        Eigen::MatrixXd P_new = P;
        Eigen::Matrix<double, 3, 3> r6t_new = R6T;

        for(const auto&[axis_index, q] : fixed_axes)
        {
            const Eigen::Matrix3d r_fixed = Eigen::AngleAxisd(q, H_new.col(axis_index).normalized()).toRotationMatrix();
            P_new.col(axis_index) += r_fixed*P_new.col(axis_index+1);
            
            P_new.block(0, axis_index+1, 3, P_new.cols()-axis_index-2) = r_fixed*P_new.block(0, axis_index+2, 3, P_new.cols()-axis_index-2).eval();

            // Only adjust r6t if last axis is locked
            if(axis_index+1 < H_new.cols())
            {
                H_new.col(axis_index) = r_fixed*H_new.col(axis_index+1);
                H_new.block(0, axis_index+1, 3, H_new.cols()-axis_index-2) = r_fixed*H_new.block(0, axis_index+2, 3, H_new.cols()-axis_index-2).eval();
            }

            H_new = H_new.block(0, 0, 3, H_new.cols()-1).eval();
            P_new = P_new.block(0, 0, 3, P_new.cols()-1).eval();
            r6t_new = r_fixed*r6t_new;
        }

        return {H_new, P_new,r6t_new};
    }
} // namespace EAIK

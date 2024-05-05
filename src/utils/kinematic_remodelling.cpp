#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <math.h>

#include "kinematic_remodelling.h"


namespace EAIK
{
    bool do_axis_intersect(const Eigen::Vector3d& h1, const Eigen::Vector3d& h2, const Eigen::Vector3d& p12, const double ZERO_THRESHOLD, const double AXIS_INTERSECT_THRESHOLD)
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

        // Remodel Robot "Base"
        for(unsigned i = 0; i < H.cols()-4; i++)
        {
            p0_i_plus1 += P_new.col(i);
            if (do_axis_intersect(H.col(i), H.col(i+1), P.col(i+1), ZERO_THRESHOLD, AXIS_INTERSECT_THRESHOLD))
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
            if (do_axis_intersect(H.col(i), H.col(i+1), P.col(i+1), ZERO_THRESHOLD, AXIS_INTERSECT_THRESHOLD))
            {
                Eigen::Vector3d intersection = calc_intersection(H.col(i), H.col(i+1), p0_i_plus1, P.col(i+1), AXIS_INTERSECT_THRESHOLD);

                P_new.col(i+1) = Eigen::Vector3d(0,0,0);
              
                unsigned j = i+2;
                Eigen::Vector3d p0j = p0_i_plus1+P.col(j-1)+P.col(j);

                while(j < H.cols() && is_point_on_Axis(H.col(j), p0j, intersection, AXIS_INTERSECT_THRESHOLD))
                {
                    P_new.col(j) = Eigen::Vector3d(0,0,0);
                    j++;
                    p0j += P.col(j);
                }
                P_new.col(i) = intersection - (p0_i_plus1-P_new.col(i));
                P_new.col(j) = p0j - intersection;
                i = j-2;
                p0_i_plus1 = p0j - P.col(j)-P.col(j-1);
            }
        }
        return P_new;
    }
} // namespace EAIK


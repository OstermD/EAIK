#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <math.h>

#include "sp.h"
#include "IKS.h"
#include "utils/kinematic_utils.h"
#include "kinematic_remodeling.h"

namespace IKS
{
    General_5R::General_5R(const Eigen::Matrix<double, 3, 5> &H, const Eigen::Matrix<double, 3, 6> &P)
        : General_Robot(H, P), H(H), P(P)
    {
    }

    General_5R::KinematicClass General_5R::determine_Kinematic_Class()
    {
        if (this->H.col(0).cross(this->H.col(1)).norm() < ZERO_THRESH 
        && this->H.col(1).cross(this->H.col(2)).norm() < ZERO_THRESH)
        {
            // h1 || h2 || h3
            if(this->H.col(3).cross(this->H.col(4)).norm() < ZERO_THRESH)
            {
                // h4 || h5
                return KinematicClass::FIRST_SECOND_THIRD_PARALLEL_FOURTH_FITH_PARALLEL;
            }
            return KinematicClass::FIRST_SECOND_THIRD_PARALLEL;
        }
        else if (this->H.col(2).cross(this->H.col(3)).norm() < ZERO_THRESH 
        && this->H.col(3).cross(this->H.col(4)).norm() < ZERO_THRESH)
        {
            // h5 || h4 || h3
            return KinematicClass::REVERSED;
        }
        
        if (this->H.col(1).cross(this->H.col(2)).norm() < ZERO_THRESH 
        && this->H.col(2).cross(this->H.col(3)).norm() < ZERO_THRESH)
        {
            // h2 || h3 || h4
            return KinematicClass::SECOND_THIRD_FOURTH_PARALLEL;
        }

        if (EAIK::do_axes_intersect(this->H.col(3), this->H.col(4), this->P.col(4), ZERO_THRESH, ZERO_THRESH))
        {
            // h4xh5
            if (EAIK::do_axes_intersect(this->H.col(2), this->H.col(3), this->P.col(3), ZERO_THRESH, ZERO_THRESH))
            {
                const Eigen::Vector3d p03 = P.block<3,3>(0,0).rowwise().sum();
                const Eigen::Vector3d intersection = EAIK::calc_intersection(H.col(2), H.col(3), p03, P.col(3), ZERO_THRESH);
    
                if(EAIK::is_point_on_Axis(H.col(4), p03+P.col(3)+P.col(4), intersection, ZERO_THRESH))
                {
                    // h3xh4 -> Spherical Wrist at end effector side
                    if (EAIK::do_axes_intersect(this->H.col(0), this->H.col(1), this->P.col(1), ZERO_THRESH, ZERO_THRESH))
                    {
                        // h1xh2
                        return KinematicClass::SPHERICAL_WRIST_FIRST_SECOND_INTERSECTING;
                    }
                    return KinematicClass::SPHERICAL_WRIST;
                }
            }

            if (EAIK::do_axes_intersect(this->H.col(1), this->H.col(2), this->P.col(2), ZERO_THRESH, ZERO_THRESH))
            {
                // h2xh3
                return KinematicClass::FOURTH_FITH_INTERSECTING_SECOND_THIRD_INTERSECTING;
            }

            if (EAIK::do_axes_intersect(this->H.col(0), this->H.col(1), this->P.col(1), ZERO_THRESH, ZERO_THRESH))
            {
                // h1xh2
                const Eigen::Vector3d intersection = EAIK::calc_intersection(H.col(0), H.col(1), this->P.col(0), P.col(1), ZERO_THRESH);
    
                if(EAIK::is_point_on_Axis(H.col(2), P.col(0)+P.col(1)+P.col(2), intersection, ZERO_THRESH))
                {
                    // h2xh3 -> Spherical Wrist at robot base
                    return KinematicClass::REVERSED;
                }
                return KinematicClass::FOURTH_FITH_INTERSECTING_FIRST_SECOND_INTERSECTING;
            }

            if (this->H.col(0).cross(this->H.col(1)).norm() < ZERO_THRESH)
            {
                // h1 || h2
                return KinematicClass::FOURTH_FITH_INTERSECTING_FIRST_SECOND_PARALLEL;
            }

            if (this->H.col(1).cross(this->H.col(2)).norm() < ZERO_THRESH)
            {
                // h2 || h3
                return KinematicClass::FOURTH_FITH_INTERSECTING_SECOND_THIRD_PARALLEL;
            }

            return KinematicClass::FOURTH_FITH_INTERSECTING;
        }
        else if (EAIK::do_axes_intersect(this->H.col(0), this->H.col(1), this->P.col(1), ZERO_THRESH, ZERO_THRESH))
        {
            return KinematicClass::REVERSED;
        }

        if (EAIK::do_axes_intersect(this->H.col(2), this->H.col(3), this->P.col(3), ZERO_THRESH, ZERO_THRESH)
        && this->H.col(1).cross(this->H.col(2)).norm() < ZERO_THRESH)
        {
            // h3xh4 and h2 || h3
            if(this->H.col(3).cross(this->H.col(4)).norm() < ZERO_THRESH)
            {
                // h4 || h5
                return KinematicClass::THIRD_FOURTH_INTERSECTING_SECOND_THIRD_PARALLEL_FOURTH_FITH_PARALLEL;
            }
            return KinematicClass::THIRD_FOURTH_INTERSECTING_SECOND_THIRD_PARALLEL;
        }

        return KinematicClass::UNKNOWN;
    }

    IK_Solution General_5R::calculate_IK(const Homogeneous_T &ee_position_orientation) const
    {
        IK_Solution solution;
        IK_Solution solution_t_12;
        const Eigen::Vector3d p_0t = ee_position_orientation.block<3, 1>(0, 3);
        const Eigen::Matrix3d r_03 = ee_position_orientation.block<3, 3>(0, 0);
        const Eigen::Vector3d p_13 = p_0t - this->P.col(0) - r_03 * this->P.col(3);

        return solution;
    }
};
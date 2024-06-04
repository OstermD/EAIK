#include <eigen3/Eigen/Dense>
#include <math.h>

#include "kinematic_utils.h"

namespace IKS
{
    Eigen::Vector3d create_normal_vector(const Eigen::Vector3d& v, const double numerical_threshold)
    {
        // Create some (numerically stable) normal vector to v 
        Eigen::Vector3d hn = v.cross(Eigen::Vector3d(1,0,0)); 

        // As v might be nearly equal to (1,0,0)
        if(hn.isZero(numerical_threshold))
        {
            hn = v.cross(Eigen::Vector3d(0,1,0));
        }

        return hn.normalized();
    }
} // namespace IKS

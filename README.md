# EAIK: A Toolbox for Efficient Analytical Inverse Kinematics by Subproblem Decomposition
## Overview
A preprint of the accompanying paper to this codebase is available [here](https://arxiv.org/abs/2409.14815).
<br>
Please also visit our [Website](https://eaik.cps.cit.tum.de) for further informations.

With this toolbox, we propose a method for automatic inverse kinematic derivation.
We exploit intersecting and parallel axes to remodel a manipulator's kinematic chain.

This allows for a hard-coded decomposition algorithm to solve its inverse kinematics by employing pre-solved subproblems.
Our approach surpasses current analytical methods in terms of usability and derivation speed without compromising computation time or the completeness of the overall solution set.

The following figure illustrates a robot with a spherical wrist and the geometric representation of a subproblem we use to solve parts of its IK:

<figure figcaption align="center">
  <img width="50%" src="https://github.com/OstermD/EAIK/raw/PyPi/Images/Titlefigure.png"/>
</figure>

We adopt the solutions and overall canonical subproblem set from [Elias et al.](https://arxiv.org/abs/2211.05737):<br>
A. J. Elias and J. T. Wen, “Ik-geo: Unified robot inverse kinematics
using subproblem decomposition” arXiv:2211.05737, 2024<br>
Please check out their publication and [implementation](https://github.com/rpiRobotics/ik-geo).


## Capabilities of this Toolbox
The current implementation supports automatic derivation of solutions for the following 6R and 3R manipulators, as well as their mirrored version (switched base and endeffector):
<br>
<figure figcaption align="center">
  <img src="https://github.com/OstermD/EAIK/raw/PyPi/Images/Kinematic_types.png"/>
</figure>

In addition, we allow the user to solve arbitrary nR manipulators that, by locking individual joints, corrspond to one of the above kinematic families.

We implement an user friendly interface for parametrizing a robot by a URDF file, DH parameters, or simply the homogeneous transformations that correspond to the joint axes placements (see examples).

If you require a vast amount of IK problems to be computed at once, we also implement a multithreaded batched version that allows you to make full use of processor.

## Dependencies and Installation
We use [Eigen 3.4](https://eigen.tuxfamily.org/index.php?title=Main_Page) for a fast implementation of the linear algebra operations within this toolbox.
Make sure you have your Eigen headers placed in their standard directory ('/usr/include/eigen3', '/usr/local/include/eigen3') - otherwise the following steps will not work for you.

We suggest using our pip-installable [PyPi package](https://pypi.org/project/EAIK/#description). Simply use the following command on your Linux machine:

```
pip install EAIK
```


## Python examples
Our goal is to make analytical IK as accessable as possible. The following example should just provide a quick insight into the simplicity in using EAIK.
You can find more elaborate examples on how to use our toolbox within the "src/eaik/examples" directory of our [GitHub Repository](https://github.com/OstermD/EAIK/tree/main).

### Simple DH Parametrization
```python
import numpy as np
from eaik.IK_DH import DhRobot

"""
Example DH parametrization + forward kinematics for some robot kinematic
"""

d = np.array([0.67183, 0.13970, 0, 0.43180, 0, 0.0565])
alpha = np.array([-np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0])
a = np.array([0,0.43180, -0.02032, 0,0,0])
bot = DhRobot(alpha, a, d)

print(bot.hasKnownDecomposition())
print(bot.fwdKin(np.array([1,1,1,1,1,1])))

```

### Robot from a URDF file and IK on random poses

```python
import numpy as np
import random
from eaik.IK_URDF import UrdfRobot
import evaluate_ik as eval

def urdf_example(path, batch_size):
    """
    Loads spherical-wrist robot from urdf, calculates IK using subproblems and checks the solution for a certian batch size
    """
    bot = UrdfRobot(path, [])
    print("Kinematic Family of the Robot: ", bot.kinematicFamily())
    print("Joint axes orientations before remodeling: \n",bot.H_original())
    print("Joint axes' reference points before remodeling: \n", bot.P_original())
    print("Joint axes orientations after remodeling: \n", bot.H_remodeled())
    print("Joint axes' reference points after remodeling: \n", bot.P_remodeled())

    # Example desired pose
    test_angles = []
    for i in range(batch_size):
        rand_angles = np.array([random.random(), random.random(), random.random(), random.random(), random.random(),random.random()])
        rand_angles *= 2*np.pi
        test_angles.append(rand_angles)
    poses = []
    for angles in test_angles:
       poses.append(bot.fwdKin(angles))
        
    sum_pos_error = np.array([0.,0.,0.])
    sum_rot_error = np.array([0.,0.,0.])
    total_num_ls = 0
    for pose in poses:
        ik_solution = bot.IK(pose)
        error_sum_pos, error_sum_rot, is_ls  = eval.evaluate_ik(bot, ik_solution, pose, np.eye(3))
        if is_ls:
            # LS solution
            total_num_ls += 1
        sum_pos_error+=error_sum_pos
        sum_rot_error+=error_sum_rot
    print("Avg. Orientation Error: ", sum_rot_error/len(poses))
    print("Avg. Position Error: ", sum_pos_error/len(poses))
    print("Number analytical: ", len(poses)-total_num_ls)
    print("Number LS: ", total_num_ls)
```
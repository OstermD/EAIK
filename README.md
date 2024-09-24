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

We implement an user friendly interface for parametrizing a robot by a URDF file, DH parameters, or simply the homogeneous transformations that correspond to the joint axes placements (see src/eaik/examples).

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
from eaik.IK_DH import Robot

"""
Example DH parametrization + forward kinematics for a random robot kinematic
"""

d = np.array([0, 0, 0, 0.56426215, 0.31625527, 0])
alpha = np.array([np.pi/2, -np.pi/2, 0, np.pi/2, np.pi/2, 0])
a = np.array([0, 0.6766692, 0.93924826, 0.99652755, 0, 0.9355382])
bot = Robot(alpha, a, d)

print(bot.hasKnownDecomposition())
print(bot.fwdKin(np.array([1,1,1,1,1,1])))

```

### Robot from a URDF file and IK on random poses

```python
import numpy as np
import random
from eaik.IK_URDF import Robot
import evaluate_ik as eval

def urdf_example(path, batch_size):
    """
    Loads spherical-wrist robot from urdf, calculates IK using subproblems and checks the solution for a certian batch size
    """

    bot = Robot(path)

    # Example desired pose
    test_angles = []
    for i in range(batch_size):
        rand_angles = np.array([random.random(), random.random(), random.random(), random.random(), random.random(),random.random()])
        rand_angles *= 2*np.pi
        test_angles.append(rand_angles)
    poses = []

    for angles in test_angles:
       poses.append(bot.fwdKin(angles))
        
    for pose in poses:
        ik_solutions = bot.IK(pose)

        # Print forward kinematics for all solutions
        for Q in ik_solutions.Q:
            pose_fwd = bot.fwdKin(Q)
            print(pose_fwd)
```
s
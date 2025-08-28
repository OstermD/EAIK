# EAIK: A Toolbox for Efficient Analytical Inverse Kinematics by Subproblem Decomposition
**Authors:** Daniel Ostermeier,
Jonathan Külz and Matthias Althoff<br><br>
This toolbox is based on the following scientific publication:<br>
D. Ostermeier, J. Külz and M. Althoff, "Automatic Geometric Decomposition for Analytical Inverse Kinematics," in IEEE Robotics and Automation Letters, vol. 10, no. 10, pp. 9964-9971, Oct. 2025
```
@ARTICLE{EAIK,
  author={Ostermeier, Daniel and K{\"u}lz, Jonathan and Althoff, Matthias},
  journal={IEEE Robotics and Automation Letters}, 
  title={Automatic Geometric Decomposition for Analytical Inverse Kinematics}, 
  year={2025},
  volume={10},
  number={10},
  pages={9964-9971},
  doi={10.1109/LRA.2025.3597897}
}
```

You may find our open-access publication [here](https://ieeexplore.ieee.org/document/11122599):

[![ieeexplore.ieee.org](https://img.shields.io/badge/RA--L-%09Automatic_Geometric_Decomposition_for_Analytical_Inverse_Kinematics-blue)](https://ieeexplore.ieee.org/document/11122599)<br>


Please cite the above work when using this code within your own projects.

Also, please visit our [website](https://eaik.cps.cit.tum.de) for additional informations.

Additional derivations that were used within this toolbox are contained in [this PDF](https://github.com/OstermD/EAIK/blob/webpage/PDFs/EAIK_extended_appendix.pdf).
Feel free to contact us if you have any questions or suggestions: 
Open up a [GitHub issue](https://github.com/OstermD/EAIK/issues).

## Overview
With this toolbox, we propose a method for automatic inverse kinematic derivation.
We exploit intersecting and parallel axes to remodel a manipulator's kinematic chain.

This allows for a hard-coded decomposition algorithm to solve its inverse kinematics by employing pre-solved subproblems.
Our approach surpasses current analytical methods in terms of usability and derivation speed without compromising computation time or the completeness of the overall solution set.

The following figure illustrates a robot with a spherical wrist and the geometric representation of a subproblem we use to solve parts of its IK:

<figure figcaption align="center">
  <img width="50%" src="https://github.com/OstermD/EAIK/raw/PyPi/Images/Titlefigure.png"/>
</figure>

We adopt the solutions and overall canonical subproblem set from Elias et al.:<br>
A. J. Elias and J. T. Wen, “IK-Geo: Unified robot inverse kinematics
using subproblem decomposition,” Mechanism and Machine Theory,
vol. 209, no. 105971, 2025.<br>
Check out their [publication](https://www.sciencedirect.com/science/article/pii/S0094114X25000606) and [implementation](https://github.com/rpiRobotics/ik-geo).


## Capabilities of this Toolbox
The current implementation supports automatic derivation of solutions for the following 6R and 5R manipulators, as well as their mirrored version (switched base and endeffector), and all non-redundant 1-4R manipulators.
In addition, we allow the user to solve arbitrary nR manipulators that, by locking individual joints, corrspond to one of the below kinematic families.

<br>
<figure figcaption align="center">
  <img src="https://github.com/OstermD/EAIK/raw/PyPi/Images/Kinematic_types.png"/>
  <figcaption>Robot configurations (without their mirrored version) that can be solved by the current EAIK implementation. NR-Robots that contain these structures as kinematic subchains are solvable if the leftover redundant joints are locked in place.
  For the 5R manipulators, all (non-redundant) specializations of the shown classes (i.e., with additional intersecting/parallel axes) are solvable as well.</figcaption>
</figure>

<br>

We implement an user friendly interface for parametrizing a robot by a URDF file, DH parameters, or simply the homogeneous transformations that correspond to the joint axes placements.

The following figure shows an overview of our interface and a superficial showcase of our method:

<figure figcaption align="center">
  <img src="https://github.com/OstermD/EAIK/raw/PyPi/Images/Poster_Method.png"/>
</figure>

If you require a vast amount of IK problems to be computed at once, we also implement a multithreaded batched version that allows you to make full use of your processor.

## Installation
## Dependencies and Installation
We use [Eigen 3.4](https://eigen.tuxfamily.org/index.php?title=Main_Page) for a fast implementation of the linear algebra operations within this toolbox.
Make sure you have your Eigen headers placed in their standard directory ('/usr/include/eigen3', '/usr/local/include/eigen3') - otherwise the following step will not work for you.

We suggest using our pip-installable [PyPI package](https://pypi.org/project/EAIK/#description). Simply use the following command on your Linux machine:

```
pip install EAIK
```



## Usage Example
We currently provide support parametrizing a robot via DH parameters, homogeneous transformations of each joint in zero-pose with respect to the basis, as well as [ROS URDF](http://wiki.ros.org/urdf) files.
Some quick examples that demonstrate the usability of our implementation are shown in the following code-snippets:

#### DH Parameters
```python
import numpy as np
from eaik.IK_DH import DhRobot

"""
Example DH parametrization + forward kinematics for a random robot kinematic
"""

d = np.array([0.67183, 0.13970, 0, 0.43180, 0, 0.0565])
alpha = np.array([-np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0])
a = np.array([0,0.43180, -0.02032, 0,0,0])
bot = Robot(alpha, a, d)

print(bot.hasKnownDecomposition())
print(bot.fwdKin(np.array([1,1,1,1,1,1])))

```

#### Robot from a URDF file and IK on random poses
```python
import numpy as np
import random
from eaik.IK_URDF import UrdfRobot
import evaluate_ik as eval


def urdf_example(path, batch_size):
    """
    Loads spherical-wrist robot from urdf, calculates IK using subproblems and checks the solution for a certian batch size
    """

    bot = UrdfRobot(path)

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
Even more examples for the python interface are available [here](https://github.com/OstermD/EAIK/tree/main/examples).
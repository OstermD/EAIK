# EAIK: A Toolbox for Efficient Analytical Inverse Kinematics by Subproblem Decomposition
## Overview
A preprint of the accompanying paper to this codebase is available [here](https://arxiv.org/abs/2409.14815).
<br>
This code resembles the implementation of the [EAIK Python toolbox](https://pypi.org/project/EAIK/). 
Please also visit our [Website](https://eaik.cps.cit.tum.de) for further informations.

With this toolbox, we propose a method for automatic inverse kinematic derivation.
We exploit intersecting and parallel axes to remodel a manipulator's kinematic chain.

This allows for a hard-coded decomposition algorithm to solve its inverse kinematics by employing pre-solved subproblems.
Our approach surpasses current analytical methods in terms of usability and derivation speed without compromising computation time or the completeness of the overall solution set.

The following figure illustrates a robot with a spherical wrist and the geometric representation of a subproblem we use to solve parts of its IK:
<figure figcaption align="center">
  <img width="50%" src="Images/Titlefigure.png"/>
</figure>

We adopt the solutions and overall canonical subproblem set from [Elias et al.](https://arxiv.org/abs/2211.05737):<br>
A. J. Elias and J. T. Wen, “Ik-geo: Unified robot inverse kinematics
using subproblem decomposition” arXiv:2211.05737, 2024<br>
Please check out their publication and [implementation](https://github.com/rpiRobotics/ik-geo).


## Capabilities of this Toolbox
The current implementation supports automatic derivation of solutions for the following 6R and 3R manipulators, as well as their mirrored version (switched base and endeffector):
<figure figcaption align="center">
  <img src="Images/Kinematic_types.png"/>
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

<br>

If you want to directly use the C++ functionality and skip the Python wrapper, feel free to use the EAIK::Robot() class of the C++ library in CPP/src.
Make sure to clone the external dependencies of this library using:
```
$ git clone --recurse-submodules git@github.com:OstermD/EAIK.git
```
Then build the EAIK C++ library by using:

```
$ mkdir EAIK/CPP/src/build && cd EAIK/CPP/src/build
$ cmake ..
$ make
```

You can build and install the python package implemented by this exact revision of the EAIK repository by using:

```
pip install .
```

## Python examples
Our goal is to make analytical IK as accessable as possible.
You can find examples on how to use our toolbox within the "src/eaik/examples" directory. 
The following examples are available:
* load_urdf.py : A simple showcase how to parse a robot's representation from a URDF file and solve it's IK
* load_urdf_7dof.py: Loads a 7R robot (e.g., the KUKA iiwa7 r800), locks one joint in place and finds analytical IK solutions for it
* batched_IK.py : Showcases how to perform multithreaded batched IK computations. Four threads are recommended.
* dh_ik_example.py : 15 lines of code that show how you can use the DH convention to parametrized robots in our environment
* evaluate_ik.py: Helper class to evaluate how the obtained solutions are

## C++ Usage
If you prefer to directly use our C++ implementation in your project, you can either parametrized your robot by the DH convention or use the internal representation.
When using latter, the EAIK::Robot() class receives two parameters: H and P.

H represents an Eigen MatrixXd with the columns corresponding to the unit vectors of each joint with respect to a global basis frame.

P represents an Eigen MatrixXd with the columns corresponding to the link-offsets.
P.col(i) therefore represents the offset between joint i and i+1 with the same orientation as the global basis frame.
This convention is consistent with that of [Elias et al.](https://github.com/rpiRobotics/ik-geo)
We suggest to take a look at the usage within the C++ tests in CPP/Tests/IK_system_tests.cpp to get a better understanding on how to use the C++ library in your project.

## Q&A
Feel free to open up a GitHub issue or a pullrequest if you have any suggestions or questions.
This project is still in development.
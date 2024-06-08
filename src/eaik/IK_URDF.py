from urchin import URDF
import numpy as np
import eaik.cpp.canonical_subproblems as cs
from time import perf_counter_ns
class Robot:
    def __urdf_to_sp_conv(axis_Trafo, axis, parent_p):
        """
        Convert urchin axis to axis-translation convention for subproblems

        :param axis: Axis object from robot (urchin)
        :param parent_rot: Rotation matrix of prior frames to calculate global rotion
        :return: (Axis vector, translation)
        """
        R = axis_Trafo[:-1, :-1]  # Rotation in global basis frame
        T = axis_Trafo[:-1, -1] - parent_p  # Translation in local joint-frame
        axis_n = R.dot(axis)
        return (axis_n, T)

    def __init__(self, file_path):
        robot = URDF.load(file_path)
        joints = robot._sort_joints(robot.actuated_joints)


        fk_zero_pose = robot.link_fk()  # Calculate FK

        parent_p = np.zeros(3)
        H = np.array([], dtype=np.int64).reshape(0, 3)  # axes
        P = np.array([], dtype=np.int64).reshape(0, 3)  # offsets
        for i in range(len(joints)):
            joint_child_link = robot.link_map[joints[i].child]
            h, p = Robot.__urdf_to_sp_conv(fk_zero_pose[joint_child_link], joints[i].axis, parent_p)
            H = np.vstack([H, h])
            P = np.vstack([P, p])
            parent_p += p

        # Endeffector displacement is (0,0,0)
        P = np.vstack([P, np.zeros(3)])
        self.__robot = cs.Robot(H.T, P.T,True)
    def hasSphericalWrist(self):
        return self.__robot.isSpherical()
    
    def fwdKin(self, Q : np.array):
        return self.__robot.fwdkin(Q)
        
    def IK(self, pose : np.ndarray):
        return self.__robot.calculate_IK(pose)
        

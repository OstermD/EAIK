import numpy as np
import csv
import copy

import eaik.cpp.canonical_subproblems as cs


class Robot:
    __ee_zero_pose_rotation = None

    def __init__(self, file_path, is_double_precision):
        with open(file_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)

            # q = None
            T01 = None
            T02 = None
            T03 = None
            T04 = None
            T05 = None
            T06 = None

            for row in reader:
                q = np.array([np.float64(row['q1']), np.float64(row['q2']), np.float64(row['q3']),
                            np.float64(row['q4']), np.float64(row['q5']), np.float64(row['q6'])])
                T01 = np.array([[np.float64(row['1-T00']), np.float64(row['1-T01']), np.float64(row['1-T02']), np.float64(row['1-T03'])],
                                [np.float64(row['1-T10']), np.float64(row['1-T11']), np.float64(row['1-T12']), np.float64(row['1-T13'])],
                                [np.float64(row['1-T20']), np.float64(row['1-T21']), np.float64(row['1-T22']), np.float64(row['1-T23'])],
                                [np.float64(row['1-T30']), np.float64(row['1-T31']), np.float64(row['1-T32']), np.float64(row['1-T33'])]])
                T02 = np.array([[np.float64(row['2-T00']), np.float64(row['2-T01']), np.float64(row['2-T02']), np.float64(row['2-T03'])],
                                [np.float64(row['2-T10']), np.float64(row['2-T11']), np.float64(row['2-T12']), np.float64(row['2-T13'])],
                                [np.float64(row['2-T20']), np.float64(row['2-T21']), np.float64(row['2-T22']), np.float64(row['2-T23'])],
                                [np.float64(row['2-T30']), np.float64(row['2-T31']), np.float64(row['2-T32']), np.float64(row['2-T33'])]])
                T03 = np.array([[np.float64(row['3-T00']), np.float64(row['3-T01']), np.float64(row['3-T02']), np.float64(row['3-T03'])],
                                [np.float64(row['3-T10']), np.float64(row['3-T11']), np.float64(row['3-T12']), np.float64(row['3-T13'])],
                                [np.float64(row['3-T20']), np.float64(row['3-T21']), np.float64(row['3-T22']), np.float64(row['3-T23'])],
                                [np.float64(row['3-T30']), np.float64(row['3-T31']), np.float64(row['3-T32']), np.float64(row['3-T33'])]])
                T04 = np.array([[np.float64(row['4-T00']), np.float64(row['4-T01']), np.float64(row['4-T02']), np.float64(row['4-T03'])],
                                [np.float64(row['4-T10']), np.float64(row['4-T11']), np.float64(row['4-T12']), np.float64(row['4-T13'])],
                                [np.float64(row['4-T20']), np.float64(row['4-T21']), np.float64(row['4-T22']), np.float64(row['4-T23'])],
                                [np.float64(row['4-T30']), np.float64(row['4-T31']), np.float64(row['4-T32']), np.float64(row['4-T33'])]])
                T05 = np.array([[np.float64(row['5-T00']), np.float64(row['5-T01']), np.float64(row['5-T02']), np.float64(row['5-T03'])],
                                [np.float64(row['5-T10']), np.float64(row['5-T11']), np.float64(row['5-T12']), np.float64(row['5-T13'])],
                                [np.float64(row['5-T20']), np.float64(row['5-T21']), np.float64(row['5-T22']), np.float64(row['5-T23'])],
                                [np.float64(row['5-T30']), np.float64(row['5-T31']), np.float64(row['5-T32']), np.float64(row['5-T33'])]])
                T06 = np.array([[np.float64(row['6-T00']), np.float64(row['6-T01']), np.float64(row['6-T02']), np.float64(row['6-T03'])],
                                [np.float64(row['6-T10']), np.float64(row['6-T11']), np.float64(row['6-T12']), np.float64(row['6-T13'])],
                                [np.float64(row['6-T20']), np.float64(row['6-T21']), np.float64(row['6-T22']), np.float64(row['6-T23'])],
                                [np.float64(row['6-T30']), np.float64(row['6-T31']), np.float64(row['6-T32']), np.float64(row['6-T33'])]])

        frame_transformations = np.array([T01, T02, T03, T04, T05, T06])

        H = np.array([], dtype=np.float64).reshape(0, 3)  # axes
        P = np.array([], dtype=np.float64).reshape(0, 3)  # offsets

        # Derive Kinematic from zero-position
        parent_p = np.zeros(3, dtype=np.float64)
        for frame in frame_transformations:
            h, p = Robot.urdf_to_sp_conv(frame, np.array([0, 0, 1]), parent_p)
            H = np.vstack([H, h])
            P = np.vstack([P, p])
            parent_p += p

        # Endeffector displacement is (0,0,0)
        P = np.vstack([P, np.zeros(3)])
        self.__ee_zero_pose_rotation = T06[:-1, :-1]  # Rotation in global basis frame

        self.__robot = cs.Robot(H.T, P.T,is_double_precision)
        return

    @staticmethod
    def urdf_to_sp_conv(axis_trafo, axis, parent_p):
        """
        Convert urchin axis to axis-translation convention for subproblems

        :param axis_trafo: TODO
        :param axis: Axis object from robot (urchin)
        :param parent_p: TODO
        :return: (Axis vector, translation)
        """
        R = axis_trafo[:-1, :-1]  # Rotation in global basis frame
        T = axis_trafo[:-1, -1] - parent_p  # Translation in local joint-frame
        axis_n = R.dot(axis)
        return axis_n, T


    def hasSphericalWrist(self):
        return self.__robot.is_spherical()

    def fwdKin(self, Q : np.array):
        pose = self.__robot.fwdkin(Q)
        pose[:-1, :-1] = pose[:-1, :-1].dot(self.__ee_zero_pose_rotation) # Account for static EE rotation
        return pose
        
    def IK(self, pose : np.ndarray):
        mod_pose = copy.deepcopy(pose)
        mod_pose[:-1, :-1] = mod_pose[:-1, :-1].dot(self.__ee_zero_pose_rotation.T) # Account for static EE rotation
        return self.__robot.calculate_IK(mod_pose)
        
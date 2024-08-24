import numpy as np

import eaik.pybindings.EAIK as EAIK


class Robot:
    def __init__(self, dh_alpha : np.ndarray, dh_a : np.ndarray, dh_d : np.ndarray, fixed_axes : list[tuple[int, float]] = [], joint_axis : np.array = np.array([0, 0, 1]), use_double_precision : bool = True):
        """
        EAIK Robot parametrized by homogeneous joint transformations

        :param dh_alpha
        :param dh_a
        :param dh_d
        :param fixed_axes: List of tuples defining fixed joints (zero-indexed) (i, q_i+1)
        :param joint_axis: Unit vector of joint axis orientation within each frame in joint_trafos (e.g., z-axis)
        :param use_double_precision: Sets numerical zero-threshold for parametrization (EAIK internally uses double precision)
        """
            
        # Use numerical zero-threshold to "stabilize" solutions for single precision accuracy (experimental)
        if not use_double_precision:
            P = np.where(np.abs(P) < 1e-5, 0, P)
            H = np.where(np.abs(H) < 1e-5, 0, H)

        self.__robot = EAIK.Robot(dh_alpha, dh_a, dh_d, np.eye(3), fixed_axes, True)
        return

    @staticmethod
    def urdf_to_sp_conv(axis_trafo, axis, parent_p):
        """
        Convert urchin axis to axis-translation convention for subproblems

        :param axis_trafo: 4x4 homogeneous transformation of a joint w.r.t. a world frame
        :param axis: Joint axis within axis_trafo (e.g., z-axis)
        :param parent_p: Linear global offset of last joint
        :return: (Axis vector, translation)
        """
        R = axis_trafo[:-1, :-1]  # Rotation in global basis frame
        T = axis_trafo[:-1, -1] - parent_p  # Translation in local joint-frame
        axis_n = R.dot(axis)
        return axis_n, T

    def hasSphericalWrist(self):
        return self.__robot.is_spherical()

    def fwdKin(self, Q : np.array):
        return self.__robot.fwdkin(Q)
        
    def IK(self, pose : np.ndarray):
        return self.__robot.calculate_IK(pose)
    
    def IK_batched(self, pose_batch, num_worker_threads=4):
        # Check if the input is a pure numpy array
        if isinstance(pose_batch, np.ndarray) and pose_batch.ndim == 3 and pose_batch.shape[1:] == (4, 4):
            pose_list = [pose_batch[i] for i in range(pose_batch.shape[0])]
        elif isinstance(pose_batch, list) and all(isinstance(p, np.ndarray) and p.shape == (4, 4) for p in pose_batch):
            pose_list = pose_batch
        else:
            raise ValueError("Input must be a numpy array with shape (batch_size, 4, 4) or a list of 2D numpy arrays with shape (4, 4)")
        return self.__robot.calculate_IK_batched(pose_list, num_worker_threads)

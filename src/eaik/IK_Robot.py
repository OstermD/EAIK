from abc import ABC

import numpy as np

import eaik.pybindings.EAIK as EAIK


class IKRobot(ABC):
    """An interface for the python-side robot implementation of this toolbox"""

    _robot: EAIK.Robot

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
        return self._robot.is_spherical()

    def hasKnownDecomposition(self):
        return self._robot.has_known_decomposition()

    def fwdKin(self, Q: np.array):
        return self._robot.fwdkin(Q)

    def IK(self, pose: np.ndarray):
        return self._robot.calculate_IK(pose)

    def IK_batched(self, pose_batch, num_worker_threads=4):
        # Check if the input is a pure numpy array
        if isinstance(pose_batch, np.ndarray) and pose_batch.ndim == 3 and pose_batch.shape[1:] == (4, 4):
            pose_list = [pose_batch[i] for i in range(pose_batch.shape[0])]
        elif isinstance(pose_batch, list) and all(isinstance(p, np.ndarray) and p.shape == (4, 4) for p in pose_batch):
            pose_list = pose_batch
        else:
            raise ValueError("Input must be a numpy array with shape (batch_size, 4, 4) or a list of 2D numpy arrays with shape (4, 4)")
        return self._robot.calculate_IK_batched(pose_list, num_worker_threads)
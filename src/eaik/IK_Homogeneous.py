import numpy as np

import eaik.pybindings.EAIK as EAIK
from eaik.IK_Robot import IKRobot


class HomogeneousRobot(IKRobot):
    """A robot parametrized by homogeneous joint transformations"""

    def __init__(self,
                 joint_trafos: np.ndarray,
                 fixed_axes: list[tuple[int, float]] = None,
                 joint_axis: np.array = np.array([0, 0, 1])):
        """
        EAIK Robot parametrized by homogeneous joint transformations

        :param joint_trafos: (N+1)x4x4 numpy array of homogeneous transformations of each of N joints w.r.t. the world
            frame, i.e. (T01, T02, ..., T0EE)
        :param fixed_axes: List of tuples defining fixed joints (zero-indexed) (i, q_i+1)
        :param joint_axis: Unit vector of joint axis orientation within each frame in joint_trafos (e.g., z-axis)
        """
        super().__init__()
        if fixed_axes is None:
            fixed_axes = []

        H = np.array([], dtype=np.float64).reshape(0, 3)  # axes
        P = np.array([], dtype=np.float64).reshape(0, 3)  # offsets

        # Derive Kinematic from zero-position
        parent_p = np.zeros(3, dtype=np.float64)
        for frame in joint_trafos[:-1]:
            h, p = IKRobot.urdf_to_sp_conv(frame, joint_axis, parent_p)
            H = np.vstack([H, h])
            P = np.vstack([P, p])
            parent_p += p
            
        # Account for end effector pose
        p_EE = joint_trafos[-1][:-1, -1] - joint_trafos[-2][:-1, -1] # Translation in local joint-frame
        P = np.vstack([P, p_EE])
        rNt = joint_trafos[-1][:-1, :-1]  # Rotation in global basis frame

        self._robot = EAIK.Robot(H.T, P.T, rNt, fixed_axes, True)

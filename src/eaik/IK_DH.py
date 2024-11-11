import numpy as np

import eaik.pybindings.EAIK as EAIK
from eaik.IK_Robot import IKRobot


class DhRobot(IKRobot):
    """A robot parameterized by standard Denavit-Hartenberg parameters"""

    def __init__(self,
                 dh_alpha: np.ndarray,
                 dh_a: np.ndarray,
                 dh_d: np.ndarray,
                 fixed_axes: list[tuple[int, float]] = None,
                 joint_axis: np.array = np.array([0, 0, 1]),
                 use_double_precision: bool = True):
        """
        EAIK Robot parametrized by homogeneous joint transformations

        :param dh_alpha
        :param dh_a
        :param dh_d
        :param fixed_axes: List of tuples defining fixed joints (zero-indexed) (i, q_i+1)
        :param joint_axis: Unit vector of joint axis orientation within each frame in joint_trafos (e.g., z-axis)
        :param use_double_precision: Sets numerical zero-threshold for parametrization (EAIK internally uses double precision)
        """
        super().__init__()
        if fixed_axes is None:
            fixed_axes = []

        self._robot = EAIK.Robot(dh_alpha, dh_a, dh_d, np.eye(3), fixed_axes, True)

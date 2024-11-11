from urchin import URDF
import numpy as np

import eaik.pybindings.EAIK as EAIK
from eaik.IK_Robot import IKRobot


class Robot(IKRobot):
    """A robot for which the kinematic chain is parsed from a URDF file."""

    def __init__(self,
                 file_path: str,
                 fixed_axes: list[tuple[int, float]] = None,
                 use_double_precision: bool = True):
        """
        EAIK Robot parametrized by URDF file

        :param file_path: Path to URDF file
        :param fixed_axes: List of tuples defining fixed joints (zero-indexed) (i, q_i+1)
        :param use_double_precision: Sets numerical zero-threshold for parametrization (EAIK internally uses double precision)
        """
        if fixed_axes is None:
            fixed_axes = []
        super().__init__()
        robot = URDF.load(file_path, lazy_load_meshes=True)
        joints = robot._sort_joints(robot.actuated_joints)

        fk_zero_pose = robot.link_fk()  # Calculate FK

        parent_p = np.zeros(3)
        H = np.array([], dtype=np.int64).reshape(0, 3)  # axes
        P = np.array([], dtype=np.int64).reshape(0, 3)  # offsets
        for i in range(len(joints)):
            joint_child_link = robot.link_map[joints[i].child]
            h, p = Robot.urdf_to_sp_conv(fk_zero_pose[joint_child_link], joints[i].axis, parent_p)
            H = np.vstack([H, h])
            P = np.vstack([P, p])
            parent_p += p
        
        # Use numerical zero-threshold to "stabilize" solutions for single precision accuracy (experimental)
        if not use_double_precision:
            P = np.where(np.abs(P) < 1e-5, 0, P)
            H = np.where(np.abs(H) < 1e-5, 0, H)

        # End effector displacement is (0,0,0)
        P = np.vstack([P, np.zeros(3)])
        self._robot = EAIK.Robot(H.T, P.T, np.eye(3), fixed_axes, use_double_precision)

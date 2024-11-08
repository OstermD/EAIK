from urchin import URDF
import numpy as np
import eaik.pybindings.EAIK as EAIK

class Robot:
    def __init__(self, file_path : str, fixed_axes : list[tuple[int, float]] = [], use_double_precision : bool = True):
        """
        EAIK Robot parametrized by URDF file

        :param file_path: Path to URDF file
        :param fixed_axes: List of tuples defining fixed joints (zero-indexed) (i, q_i+1)
        :param use_double_precision: Sets numerical zero-threshold for parametrization (EAIK internally uses double precision)
        """
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

        # Endeffector displacement is (0,0,0)
        P = np.vstack([P, np.zeros(3)])
        self.__robot = EAIK.Robot(H.T, P.T, np.eye(3), fixed_axes, use_double_precision)
        
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

    @property
    def H(self) -> np.ndarray:
        """Returns the joint axes of this robot"""
        return self.__robot.bot_kinematics.H

    @property
    def P(self) -> np.ndarray:
        """Returns the reference points for this robot"""
        return self.__robot.bot_kinematics.P
    
    def hasSphericalWrist(self):
        return self.__robot.isSpherical()
    
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

###
#
# Large parts of this code stem from the spatial.py file in the "Timor Python" Repository
# 
# https://gitlab.lrz.de/tum-cps/timor-python
#
###
import numpy as np
from scipy.spatial.transform import Rotation

NO_TRANSLATION = np.zeros([3], float)
NO_ROTATION = np.eye(3)
NO_TRANSLATION.setflags(write=False)
NO_ROTATION.setflags(write=False)

def axis_angle2rot_mat(axis_angle: np.ndarray) -> np.ndarray:
    """
    Inverse to rot_mat2axis_angle.
    """
    rot_vec = axis_angle[:3] * axis_angle[3]
    return Rotation.from_rotvec(rot_vec).as_matrix()  # Calling copy to being able working with read-only arrays

def vecs2RotMat(sourceVector: np.ndarray, destVector: np.ndarray):
    """
    Calculate 3x3 rotation matrix for correspondence between two vectors

    :param sourceVector: start vector
    :param destVector: desired position
    :return:  3x3 rotation matrix
    """
    normSource = sourceVector / np.linalg.norm(sourceVector)
    normDest = destVector / np.linalg.norm(destVector)

    # Get axis angle representation https://www.euclideanspace.com/maths/algebra/vectors/angleBetween
    theta = np.arccos(normSource.dot(normDest))

    if np.isclose(theta, 0):
        # sourceVector = destVector
        return np.eye(3)

    rotVec = np.cross(normSource, normDest)
    if np.isclose(np.linalg.norm(rotVec), 0):
        # Rot-Vec is null-vector -> use arbitrary orthogonal vector of sourceVec
        orthogonal = sourceVector
        orthogonal[[0, 1]] = orthogonal[[1, 0]]
        orthogonal[0] = -orthogonal[0]
        return axis_angle2rot_mat(np.append(orthogonal, np.array([theta])))

    normRotVec = rotVec / np.linalg.norm(rotVec)
    return axis_angle2rot_mat(np.append(normRotVec, np.array([theta])))

def mat2euler(R: np.ndarray, seq: str = 'xyz') -> np.ndarray:
    """
    Inverse to euler2mat

    :param R: A 3x3 rotation matrix
    :param seq: Any ordering of {xyz}[intrinsic] or {XYZ}[extrinsic] axes. Defaults to roll-pitch-yaw
    :return: The rotations around the axes specified in seq that lead to R
    """
    if seq == 'xyz':
        shortcuts = (
            (NO_ROTATION, np.zeros(3)),
            (np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]), np.array([np.pi, 0, 0])),
        )
        for shortcut, angles in shortcuts:  # This saves a lot of time for the common cases
            if (R == shortcut).all():
                return angles
    return Rotation.from_matrix(R.copy()).as_euler(seq)  # Calling copy to being able working with read-only arrays


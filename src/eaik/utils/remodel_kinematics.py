import numpy as np
import utils.spherical_wrist_checks as sph
import utils.frame_trafo_helpers as trafo

def remodel_intersection(H, P, j):
    """
    Remodel individual axis intersections such that frames of intersecting axes coincide.

    :param H: Joint axes
    :param P: Link displacements between joints
    :param P: First intersection axis
    :return: P_new
    """
    P_new = P.copy()
    # Remodel wrist
    # Start with third last axis
    k = j - 1  # Previous axis to j
    e = j + 1  # Following axis to j

    p_0_k = np.sum(P[0:k + 1, :], 0)
    p_0_j = np.sum(P[0:j + 1, :], 0)
    p_0_e_1 = np.sum(P[0:e + 2, :], 0)  # p_0_(e+1)
    T_0_j = np.vstack([np.vstack([np.eye(3), p_0_j]).T, np.array([0, 0, 0, 1])])

    # Transform j axis into xyz-rpy convention with X-Axis as joint axis
    _, rpy_1, _ = sph.transformToNewAxisConvention(T_0_j, H[j], np.array([1., 0., 0.]), np.eye(3))
    axis_1_rot_mat = trafo.get_rpy_rot_matrix(rpy_1)

    # Transformation in new frame where X-axis of xyz_1 is joint axis
    T_j_e = np.vstack([np.vstack([axis_1_rot_mat, axis_1_rot_mat.dot(P[j + 1])]).T, np.array([0, 0, 0, 1])])

    # Transform e axis into xyz-rpy convention with X-Axis as joint axis
    xyz_2, rpy_2, _ = sph.transformToNewAxisConvention(T_j_e, H[e], np.array([1., 0., 0.]), np.eye(3))

    # Intersection point w.r.t. axis j in xyz-rpy convention with X-Axis as joint axis
    intersection_axis_1_frame = sph.calculate_intersection(xyz_2, rpy_2)

    # Calculate inverse rpy to yield intersection-point in world-frame
    inv_rot_mat = axis_1_rot_mat.T
    intersection_world = inv_rot_mat.dot(intersection_axis_1_frame) + p_0_j

    # Move origin of first intersection axis to intersection point
    P_new[k + 1] = intersection_world - p_0_k
    P_new[j + 1] = np.zeros(3)  # Move origin of second axis
    P_new[e + 1] = p_0_e_1 - intersection_world
    return P_new


def remodel_sp_kinematics(H: np.array, P: np.array):
    """
    Remodel whole kinematic struct such that frames of intersecting axes coincide.

    :param H: Joint axes
    :param P: Link displacements between joints
    :return: P_new
    """
    P_new = P.copy()

    # Remodel base
    for i in range(len(H) - 4):
        # Axes must not be parallel
        if not np.isclose(np.linalg.norm(np.cross(H[i], H[i + 1])), 0):
            # Check if axes intersect
            if np.isclose(np.cross(H[i], H[i + 1]).dot(P[i + 1]), 0):
                P_new = remodel_intersection(H, P_new, i)

    # Remodel wrist
    for i in range(len(H) - 3, len(H) - 1):
        # Axes must not be parallel
        if not np.isclose(np.linalg.norm(np.cross(H[i], H[i + 1])), 0):
            # Check if axes intersect
            if np.isclose(np.cross(H[i], H[i + 1]).dot(P[i + 1]), 0):
                P_new = remodel_intersection(H, P_new, i)
    return P_new
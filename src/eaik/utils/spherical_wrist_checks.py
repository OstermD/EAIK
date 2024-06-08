import numpy as np
import eaik.utils.frame_trafo_helpers as trafo
import timor.utilities.spatial as spat
from urchin import URDF


def transformToNewAxisConvention(frameTrafo, axis, newAxis, parentFrameBasisChange):
    """
    Transforms an arbitrary coordinate frame to one with a different joint axis

    :param frameTrafo: 4x4 homogeneous transformation matrix of frame
    :param axis: current joint axis
    :param newAxis: desired new joint axis
    :param parentFrameBasisChange: coordinate basis change of last axis for coordinate transformation
    :return:    xyz, rpy, frame basis
    """
    invParentFrameBasis = np.linalg.inv(parentFrameBasisChange)
    R = frameTrafo[:-1, :-1]
    T = frameTrafo[:-1, -1]
    R_newBasis = invParentFrameBasis.dot(R)
    T_newBasis = invParentFrameBasis.dot(T)

    axisTrafo = spat.vecs2RotMat(axis, newAxis)
    newRpy = R_newBasis.dot(axisTrafo)

    euler_angles = spat.mat2euler(newRpy, "xyz")
    return T_newBasis, euler_angles, axisTrafo


def calculate_intersection(xyz: np.array,
                           rpy: np.array):
    """
    Calculate intersection w.r.t. x_0

    :param xyz: numpy array of x,y,z coordinates
    :param rpy: numpy array of roll, pitch, yaw angles in radians in that orde
    :return:    Coordinates of intersection
    """
    if not check_first_intersection_axis(xyz, rpy):
        raise Exception("Second axis is doesn't intersect first!")

    _, beta, gamma = rpy[0], rpy[1], rpy[2]

    x, y, z = xyz[0], xyz[1], xyz[2]
    if np.isclose(0, abs(beta) % np.pi, atol=tol):
        # z =/= 0 as intersection exists according to check_first_intersection_axis
        if np.isclose(0, abs(gamma) % np.pi, atol=tol):
            return np.array([np.inf, np.inf, np.inf])
        else:
            return np.array([((-y) / np.tan(gamma)) + x, 0, 0])
    return np.array([((np.cos(gamma) * z) / np.tan(beta)) + x, 0, 0])




def check_second_intersection_axis(xyz_1: np.array,
                                   rpy_1: np.array,
                                   xyz_2: np.array,
                                   rpy_2: np.array):
    """
    Check if second axis intersecting axis 1 and axis 0

    :param xyz_1: numpy array of x,y,z coordinates of first axis
    :param rpy_1: numpy array of roll, pitch, yaw angles in radians in that order of frame 1
    :param xyz_2: numpy array of x,y,z coordinates of second axis
    :param rpy_2: numpy array of roll, pitch, yaw angles in radians in that order of frame 2
    :return:    True/False
    """
    if not check_first_intersection_axis(xyz_1, rpy_1):
        return False

    # Calculate intersection point
    intersec = calculate_intersection(xyz_1, rpy_1)
    if np.Inf in intersec:
        # Axis 0 and 1 are the same axis
        return check_first_intersection_axis(xyz_2, rpy_2)
    
    # Intersection in basis of the coordinate frame of axis 1
    mue = np.linalg.inv(trafo.get_rpy_rot_matrix(rpy_1)).dot(intersec - xyz_1)
    _, beta_2, gamma_2 = rpy_2[0], rpy_2[1], rpy_2[2]
    x_2, y_2, z_2 = xyz_2[0], xyz_2[1], xyz_2[2]

    if np.isclose(0, abs(beta_2) % np.pi, atol=tol):
        if np.isclose(z_2, 0, atol=tol):
            if np.isclose(abs(gamma_2) % np.pi, 0, atol=tol):
                raise Exception("Last two axes are redundant!")
            condition_0 = ((-y_2) / np.tan(gamma_2)) + x_2 - mue[0]
            return np.isclose(condition_0, 0, atol=tol)
        return False

    lambda_ = z_2 / np.sin(beta_2)
    condition_1 = np.cos(beta_2) * np.sin(gamma_2) * lambda_ + y_2
    condition_2 = np.cos(gamma_2) * np.cos(beta_2) * lambda_ + x_2 - mue[0]
    return np.isclose(condition_1, 0, atol=tol) and np.isclose(condition_2, 0, atol=tol)


def poe_check_spherical_wrist(P):
    """
    Check if the provided POE contains a spherical wrist.

    :param P: Remodeled link diplacement
    :return:    True/False
    """
    #e.g.: P_45 and P_56 must be zero vectors in case of spherical wrist in a 6R manipulator
    return np.allclose(P[-3], np.zeros(3)) and np.allclose(P[-2], np.zeros(3))

def urdf_check_spherical_wrist(path: str):
    """
    Loads urdf file from given path and checks if a spherical wrist exists

    :param path: path to urdf file
    :return:    True/False
    """
    robot = URDF.load(path)
    wrist_joints = robot._sort_joints(robot.actuated_joints)[-3:]
    axis_0 = wrist_joints[-3]
    axis_1 = wrist_joints[-2]
    axis_2 = wrist_joints[-1]

    xyz_1, rpy_1, basis_1 = transformToNewAxisConvention(
        axis_1.origin, axis_1.axis, np.array([1, 0, 0]), spat.vecs2RotMat(np.array([1, 0, 0]), axis_0.axis))

    xyz_2, rpy_2, basis_2 = transformToNewAxisConvention(
        axis_2.origin, axis_2.axis, np.array([1, 0, 0]), basis_1)

    num_joints = len(robot.actuated_joints)
    return check_second_intersection_axis(xyz_1, rpy_1, xyz_2, rpy_2), num_joints
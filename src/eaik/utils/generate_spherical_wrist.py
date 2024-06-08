import numpy as np
import random

import eaik.utils.frame_trafo_helpers as trafo

tol = 1e-7


def check_first_intersection_axis(xyz: np.array,
                                  rpy: np.array):
    """
    Check first axis intersection with x_0

    :param xyz: numpy array of x,y,z coordinates
    :param rpy: numpy array of roll, pitch, yaw angles in radians in that order
    :return:    True/False
    """
    _, beta, gamma = rpy[0], rpy[1], rpy[2]
    _, y, z = xyz[0], xyz[1], xyz[2]
    if np.isclose(0, abs(beta) % np.pi, atol=tol):
        if np.isclose(z, 0, atol=tol):
            if np.isclose(abs(gamma) % np.pi, 0, atol=tol):
                if np.isclose(y, 0, atol=tol):
                    return True  # Axis 2 == Axis 1
                return False  # Offset in y-direction with yaw=0
            return True  # Axis 2 not parallel to Axis 1
        return False  # Offset in z-direction with pitch=0

    lambda_ = z / np.sin(beta)
    return np.isclose(np.cos(beta) * np.sin(gamma) * lambda_ + y, 0, atol=tol)


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

def generate_first_intersection_axis(max_translation=1, translation_digit_acc=3, rotation_digit_acc=2):
    """
    Generate axis intersecting x_0=(1,0,0)

    :param workspace_radius: radius for maximum translation in each direction
    :param translation_digit_acc: Decimal places for translation values
    :param rotation_digit_acc:  Decimal places for rotation values
    :return: Tuple of numpy arrays (xyz, rpy) - Rotation in radians
    """
    # random floats in [-workspace_radius, workspace_radius] rounded to three digits
    x_1 = round(random.uniform(-max_translation, max_translation), translation_digit_acc)
    y_1 = round(random.uniform(-max_translation, max_translation), translation_digit_acc)
    z_1 = round(random.uniform(-max_translation, max_translation), translation_digit_acc)
    alpha_1 = round(random.uniform(0, 2 * np.pi), rotation_digit_acc)

    beta_1 = 0
    gamma_1 = 0

    # Orthogonal to x_0 ?
    if y_1 == 0:
        if z_1 == 0:
            beta_1 = round(random.uniform(0, 2 * np.pi), rotation_digit_acc)
            gamma_1 = round(random.uniform(0, 2 * np.pi), rotation_digit_acc)

            if (beta_1 % np.pi) == 0:
                # Assure x_0 =/= x_1 (Would induce inherent redundancy)
                while (gamma_1 % np.pi) == 0:
                    gamma_1 = round(random.uniform(0, 2 * np.pi), rotation_digit_acc)
        else:
            gamma_1 = random.randint(0, 2) * np.pi
            beta_1 = round(random.uniform(0, 2 * np.pi), rotation_digit_acc)

            # Assure x_0 =/= x_1 (Would induce inherent redundancy)
            while (beta_1 % np.pi) == 0:
                beta_1 = round(random.uniform(0, 2 * np.pi), rotation_digit_acc)
    elif z_1 == 0:
        beta_1 = random.randint(0, 2) * np.pi

        gamma_1 = round(random.uniform(0, 2 * np.pi), rotation_digit_acc)
        # Assure x_0 =/= x_1 (Would induce inherent redundancy)
        while (gamma_1 % np.pi) == 0:
            gamma_1 = round(random.uniform(0, 2 * np.pi), rotation_digit_acc)
    else:
        gamma_1 = round(random.uniform(0, 2 * np.pi), rotation_digit_acc)
        # Assure point of intersection with x_0 exists
        while (gamma_1 % np.pi) == 0:
            gamma_1 = round(random.uniform(0, 2 * np.pi), rotation_digit_acc)

        beta_1 = np.arctan2(-np.sin(gamma_1) * z_1, y_1)
    return np.array([x_1, y_1, z_1]), np.array([alpha_1, beta_1, gamma_1])


# TODO:
def check_matches_x_0(xyz, rpy):
    """
    Check if axis matches x_0=(1,0,0) w.r.t. x_0 frame

    :param xyz: numpy array for translation values
    :param rpy: numpy array for rotation values
    :return: True/False
    """
    return False


def generate_second_intersection_axis(xyz_1: np.array,
                                      rpy_1: np.array,
                                      workspace_radius=1,
                                      translation_digit_acc=3,
                                      rotation_digit_acc=2):
    """
    Generate axis intersecting x_1=(1,0,0)

    :param xyz_1: translation values of axis 1
    :param rpy_1: rotation values of axis 1
    :return: Tuple of numpy arrays (xyz, rpy) - Rotation in radians
    """
    # random floats in [-workspace_radius, workspace_radius] rounded to three digits
    x_2 = round(random.uniform(-workspace_radius, workspace_radius), translation_digit_acc)
    y_2 = round(random.uniform(-workspace_radius, workspace_radius), translation_digit_acc)
    z_2 = round(random.uniform(-workspace_radius, workspace_radius), translation_digit_acc)
    alpha_2 = round(random.uniform(0, 2 * np.pi), rotation_digit_acc)

    gamma_2 = 0
    beta_2 = 0

    _, y_1, z_1 = xyz_1[0], xyz_1[1], xyz_1[2]

    intersec = calculate_intersection(xyz_1, rpy_1)

    # Intersection in basis of x_1
    mu = np.linalg.inv(trafo.get_rpy_rot_matrix(rpy_1)).dot(intersec - xyz_1)

    # Just for tests:
    if (not np.isclose(0, mu[1])) or (not np.isclose(0, mu[2])):
        raise Exception("Intersection w.r.t x_1 axis must be 0 in y_1 and z_1!")

    if (not np.isclose(0, y_2)) and (not np.isclose(0, z_2)):
        gamma_2 = np.arctan2(-y_2, mu[0] - x_2)
        beta_2 = np.arctan2(-np.sin(gamma_2) * z_2, y_2)
    elif np.isclose(0, y_2):
        if np.isclose(0, z_2):
            x_2 = mu[0]
            beta_2 = round(random.uniform(0, 2 * np.pi), rotation_digit_acc)
            gamma_2 = round(random.uniform(0, 2 * np.pi), rotation_digit_acc)

            if np.isclose(0, abs(beta_2) % np.pi):
                # Assure x_1 =/= x_2 (Would induce inherent redundancy)
                while np.isclose(0, abs(gamma_2) % np.pi):
                    gamma_2 = round(random.uniform(0, 2 * np.pi), rotation_digit_acc)
        else:
            if np.isclose(0, y_1):
                # TODO: Prove: Three axis in one plane -> Redundancy!
                print("Three axes in one plane - redundancy -> Retrying!")
                return generate_second_intersection_axis(
                    xyz_1,
                    rpy_1,
                    workspace_radius,
                    translation_digit_acc,
                    rotation_digit_acc
                )
            gamma_2 = 0  # TODO: This may also be np.pi, but shouldn't make a difference
            beta_2 = np.arctan2(z_2, mu[0] - x_2)  # Results in beta_2 = np.pi/2 in case mu[0]==x_2
    else:
        if np.isclose(0, z_1):
            # TODO: Prove: Three axis in one plane -> Redundancy!
            print("Three axes in one plane - redundancy -> Retrying!")
            return generate_second_intersection_axis(
                xyz_1,
                rpy_1,
                workspace_radius,
                translation_digit_acc, rotation_digit_acc
            )
        beta_2 = 0
        gamma_2 = np.pi - np.arctan2(y_2, mu[0] - x_2)  # Results in gamma_2 = np.pi/2 in case mu[0]==x_2

    # Third axis may not match first (Would induce inherent redundancy)
    if check_matches_x_0(np.array([x_2, y_2, z_2]), np.array([alpha_2, beta_2, gamma_2])):
        print("Third axis matches first - retrying!")
        return generate_second_intersection_axis(
            xyz_1, rpy_1, workspace_radius,
            translation_digit_acc, rotation_digit_acc
        )

    return np.array([x_2, y_2, z_2]), np.array([alpha_2, beta_2, gamma_2])

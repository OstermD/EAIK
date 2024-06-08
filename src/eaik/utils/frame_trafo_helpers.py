import numpy as np


def get_rpy_rot_matrix(rpy: np.array):
    """
    Generate Roll-Pitch-Yaw rotation matrix in radiants
    """
    alpha, beta, gamma = rpy[0], rpy[1], rpy[2]
    return np.array([
        [np.cos(gamma) * np.cos(beta), np.cos(gamma) * np.sin(beta) * np.sin(alpha) - np.cos(alpha)
         * np.sin(gamma), np.sin(gamma) * np.sin(alpha) + np.cos(gamma) * np.cos(alpha) * np.sin(beta)],
        [np.cos(beta) * np.sin(gamma), np.cos(gamma) * np.cos(alpha) + np.sin(gamma) * np.sin(beta)
         * np.sin(alpha), np.cos(alpha) * np.sin(gamma) * np.sin(beta) - np.cos(gamma) * np.sin(alpha)],
        [-np.sin(beta), np.cos(beta) * np.sin(alpha), np.cos(beta) * np.cos(alpha)]])

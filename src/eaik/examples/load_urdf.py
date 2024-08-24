import numpy as np
import random

from eaik.IK_URDF import Robot
import evaluate_ik as eval

def test_urdf(path, batch_size):
    """
    Loads spherical-wrist robot from urdf, calculates IK using subproblems and checks the solution for a certian batch size
    """
    bot = Robot(path)

    # Example desired pose
    test_angles = []
    for i in range(batch_size):
        rand_angles = np.array([random.random(), random.random(), random.random(), random.random(), random.random(), random.random()])
        rand_angles *= 2*np.pi
        test_angles.append(rand_angles)
    poses = []
    for angles in test_angles:
       poses.append(bot.fwdKin(angles))

    error_sum = 0
    num_no_solution = 0
    total_num_ls = 0
    total_num_analytic = 0
    for pose in poses:
        ik_solution = bot.IK(pose)

        avg_error, num_analytic, is_ls  = eval.evaluate_ik(bot, ik_solution, pose, np.eye(3))
        if is_ls:
            # LS solution
            total_num_ls += 1
            error_sum += avg_error
        elif num_analytic > 0:
            # Analytic solutions
            error_sum += avg_error
            total_num_analytic += 1
        else:
            num_no_solution += 1
    if total_num_analytic + total_num_ls > 0:
        print("Average error: ", error_sum / (total_num_analytic + total_num_ls))
    print("Number success: ", total_num_analytic + total_num_ls)
    print("Number failure: ", num_no_solution)
    print("Number LS: ", total_num_ls)

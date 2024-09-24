import numpy as np
import random
from eaik.IK_URDF import Robot
import evaluate_ik as eval

def ndof_example(path, batch_size):
    """
    Loads 7dof spherical-wrist robot from urdf, calculates IK using subproblems and checks the solution for a certian batch size
    """
    
    # Lock axis 3 (index 2) at q3=2.14
    q3 = 2.14
    bot = Robot(path, [(2, q3)], False)

    # Example desired pose
    test_angles = []
    for i in range(batch_size):
        rand_angles = np.array([random.random(), random.random(), q3, random.random(), random.random(), random.random(), random.random()])
        rand_angles *= 2*np.pi
        test_angles.append(rand_angles)
    poses = []
    for angles in test_angles:
       poses.append(bot.fwdKin(angles))

    sum_pos_error = np.array([0.,0.,0.])
    total_num_ls = 0
    for pose in poses:
        ik_solution = bot.IK(pose)
        error_sum_pos, error_sum_rot, is_ls  = eval.evaluate_ik(bot, ik_solution, pose, np.eye(3))
        if is_ls:
            # LS solution
            total_num_ls += 1
        sum_pos_error+=error_sum_pos
    print("Avg. Position Error: ", sum_pos_error/len(poses))
    print("Number analytical: ", len(poses)-total_num_ls)
    print("Number LS: ", total_num_ls)

ndof_example("lbr_iiwa7_r800.urdf", 100)
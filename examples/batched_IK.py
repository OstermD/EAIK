import numpy as np
import random
from eaik.IK_URDF import UrdfRobot
import evaluate_ik as eval

def batched_ik_example(path, batch_size):
    """
    Loads spherical-wrist robot from urdf, calculates IK using subproblems and checks the solution for a certian batch size
    """
    bot = UrdfRobot(path)

    # Example desired pose
    test_angles = []
    for i in range(batch_size):
        rand_angles = np.array([random.random(), random.random(), random.random(), random.random(), random.random(), random.random()])
        rand_angles *= 2*np.pi
        test_angles.append(rand_angles)
    poses = []
    for angles in test_angles:
       poses.append(bot.fwdKin(angles))

    # Perform batched IK computation
    solutions = bot.IK_batched(poses, num_worker_threads=4)
    
    sum_pos_error = np.array([0.,0.,0.])
    total_num_ls = 0
    for i,ik_solution in enumerate(solutions):
        error_sum_pos, error_sum_rot, is_ls  = eval.evaluate_ik(bot, ik_solution, poses[i], np.eye(3))
        if is_ls:
            # LS solution
            total_num_ls += 1
        sum_pos_error+=error_sum_pos
    print("Avg. Position Error: ", sum_pos_error/len(poses))
    print("Number analytical: ", len(poses)-total_num_ls)
    print("Number LS: ", total_num_ls)
    
batched_ik_example("Puma560.urdf", 5000)

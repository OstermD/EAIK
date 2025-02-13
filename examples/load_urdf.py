import numpy as np
import random
from eaik.IK_URDF import UrdfRobot
import evaluate_ik as eval

def urdf_example(path, batch_size):
    """
    Loads spherical-wrist robot from urdf, calculates IK using subproblems and checks the solution for a certian batch size
    """
    bot = UrdfRobot(path, [])
    print("Kinematic Family of the Robot: ", bot.kinematicFamily())
    print("Joint axes orientations before remodeling: \n",bot.H_original())
    print("Joint axes' reference points before remodeling: \n", bot.P_original())
    print("Joint axes orientations after remodeling: \n", bot.H_remodeled())
    print("Joint axes' reference points after remodeling: \n", bot.P_remodeled())

    # Example desired pose
    test_angles = []
    for i in range(batch_size):
        rand_angles = np.array([random.random(), random.random(), random.random(), random.random(), random.random(),random.random()])
        rand_angles *= 2*np.pi
        test_angles.append(rand_angles)
    poses = []
    for angles in test_angles:
       poses.append(bot.fwdKin(angles))
        
    sum_pos_error = np.array([0.,0.,0.])
    sum_rot_error = np.array([0.,0.,0.])
    total_num_ls = 0
    for pose in poses:
        ik_solution = bot.IK(pose)
        error_sum_pos, error_sum_rot, is_ls  = eval.evaluate_ik(bot, ik_solution, pose, np.eye(3))
        if is_ls:
            # LS solution
            total_num_ls += 1
        sum_pos_error+=error_sum_pos
        sum_rot_error+=error_sum_rot
    print("Avg. Orientation Error: ", sum_rot_error/len(poses))
    print("Avg. Position Error: ", sum_pos_error/len(poses))
    print("Number analytical: ", len(poses)-total_num_ls)
    print("Number LS: ", total_num_ls)
            
urdf_example("Puma560.urdf", 500)

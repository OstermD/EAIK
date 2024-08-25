import numpy as np
import random
import csv
from eaik.IK_URDF import Robot
import evaluate_ik as eval
import time

def test_urdf(path, batch_size):
    """
    Loads spherical-wrist robot from urdf, calculates IK using subproblems and checks the solution for a certian batch size
    """
    bot = Robot(path, [], False)

    # Example desired pose
    test_angles = []
    for i in range(batch_size):
        rand_angles = np.array([random.random(), random.random(), random.random(), random.random(), random.random(),random.random()])
        rand_angles *= 2*np.pi
        test_angles.append(rand_angles)
    poses = []
    for angles in test_angles:
       poses.append(bot.fwdKin(angles))

    # Cache warmup
    for i in range(1):
        ik_solution = bot.IK(poses[0])
        
    errors = []
    for pose in poses:
        start = time.perf_counter_ns()
        for i in range(1):
            ik_solution = bot.IK(pose)
            errors.append(eval.evaluate_ik(bot, ik_solution, pose, np.eye(3)))
    

    with open("/home/daniel/Documents/EAIK/Testing/stability_ur5_1e6.csv", 'w', newline='') as f:
        writer = csv.writer(f)
        
        # Write the header
        writer.writerow(["Position_Error", "Orientation_Error"])
        
        # Write the rows 
        for err in errors:
            position_err = np.linalg.norm(err[0])
            orientation_err = np.linalg.norm(err[1])
            writer.writerow([f"{position_err:.17f}", f"{orientation_err:.17f}"])
test_urdf("/home/daniel/Documents/EAIK/Testing/Puma560.urdf", 500)

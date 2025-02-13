import numpy as np
from scipy.spatial.transform import Rotation as R

def rotation_difference_vector(R1, R2):
    # Calculate the relative rotation matrix
    R_rel = np.dot(R2, R1.T)
    
    # Convert the relative rotation matrix to axis-angle representation
    rotation = R.from_matrix(R_rel)
    axis_angle = rotation.as_rotvec()

    return axis_angle

def evaluate_ik(bot, ik_solutions, groundtruth_pose, ee_zero_pose_rotation):
    orientation_should = groundtruth_pose[:-1, :-1]
    translation_should = groundtruth_pose[:-1, -1]
            
    error_sum_pos= np.zeros(3)
    error_sum_rot= np.zeros(3)
    
    num_analytic = 0
    is_ls = False
    analytic_solutions = []
    ls_solutions = []
    for i in range(len(ik_solutions.Q)):
        if (not ik_solutions.is_LS[i]):
            analytic_solutions.append(ik_solutions.Q[i])
        else:
            ls_solutions.append(ik_solutions.Q[i])

    if len(analytic_solutions) == 0:
        smallest_ls_error = np.inf
        smallest_ls_solution = None
        for sol in ls_solutions:
            res = bot.fwdKin(sol)
            is_ls = True
            # Account for init nullposition rotation
            res[:-1, :-1] = res[:-1, :-1].dot(ee_zero_pose_rotation)
            orientation_is = res[:-1, :-1]
            translation_is = res[:-1, -1]
            
            ls_error = np.linalg.norm(res - groundtruth_pose)
            if(smallest_ls_error > ls_error):
                # New best LS solution
                smallest_ls_error = ls_error
                smallest_ls_solution = sol
        if smallest_ls_solution is not None:
            res = bot.fwdKin(smallest_ls_solution)
            res[:-1, :-1] = res[:-1, :-1].dot(ee_zero_pose_rotation)
            orientation_is = res[:-1, :-1]
            translation_is = res[:-1, -1]
            
            error_sum_pos += np.abs(translation_is-translation_should)
            error_sum_rot += np.abs(rotation_difference_vector(orientation_should,orientation_is))   
        else:
            print("No Solution")
            return None
    else:
        for sol in analytic_solutions:
            res = bot.fwdKin(sol)
            # Account for init nullposition rotation
            res[:-1, :-1] = res[:-1, :-1].dot(ee_zero_pose_rotation)
            orientation_is = res[:-1, :-1]
            translation_is = res[:-1, -1]
            
            error_sum_pos += np.abs(translation_is-translation_should)
            error_sum_rot += np.abs(rotation_difference_vector(orientation_should,orientation_is))   
            num_analytic+=1

    return error_sum_pos, error_sum_rot, is_ls
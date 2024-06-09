import numpy as np

LS_thershold_error = 1e-6
def evaluate_ik(bot, ik_solutions, groundtruth_pose, ee_zero_pose_rotation):
    error_sum = 0
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
            ls_error = np.linalg.norm(res - groundtruth_pose)
            if(smallest_ls_error > ls_error):
                # New best LS solution
                smallest_ls_error = ls_error
                smallest_ls_solution = sol
        if smallest_ls_solution is not None:
            if(smallest_ls_error < LS_thershold_error):
                error_sum += smallest_ls_error
        #else:
            #print("No Solution")
    else:
        for sol in analytic_solutions:
            res = bot.fwdKin(sol)
            # Account for init nullposition rotation
            res[:-1, :-1] = res[:-1, :-1].dot(ee_zero_pose_rotation)
            error_sum += np.linalg.norm(res - groundtruth_pose)
            num_analytic+=1

    avg_error = np.inf
    if num_analytic > 0:
        avg_error = error_sum / num_analytic
    elif is_ls:
        avg_error = error_sum

    return avg_error, num_analytic, is_ls 
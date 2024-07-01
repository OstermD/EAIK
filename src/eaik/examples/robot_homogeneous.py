import evaluate_ik as eval
from eaik.IK_Homogeneous import Robot
import numpy as np
import csv

def load_test_csv(path):
    """
    Loads test robots from csv
    """
    total_num_ls = 0
    error_sum = 0
    total_num_analytic = 0
    num_no_solution = 0
    with open(path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)

        first_line = True
        for row in reader:
            T01 = np.array([[np.float64(row['1-T00']), np.float64(row['1-T01']), np.float64(row['1-T02']), np.float64(row['1-T03'])],
                            [np.float64(row['1-T10']), np.float64(row['1-T11']), np.float64(row['1-T12']), np.float64(row['1-T13'])], 
                            [np.float64(row['1-T20']), np.float64(row['1-T21']), np.float64(row['1-T22']), np.float64(row['1-T23'])],
                            [np.float64(row['1-T30']), np.float64(row['1-T31']), np.float64(row['1-T32']), np.float64(row['1-T33'])]])
            T02 = np.array([[np.float64(row['2-T00']), np.float64(row['2-T01']), np.float64(row['2-T02']), np.float64(row['2-T03'])],
                            [np.float64(row['2-T10']), np.float64(row['2-T11']), np.float64(row['2-T12']), np.float64(row['2-T13'])], 
                            [np.float64(row['2-T20']), np.float64(row['2-T21']), np.float64(row['2-T22']), np.float64(row['2-T23'])],
                            [np.float64(row['2-T30']), np.float64(row['2-T31']), np.float64(row['2-T32']), np.float64(row['2-T33'])]])
            T03 = np.array([[np.float64(row['3-T00']), np.float64(row['3-T01']), np.float64(row['3-T02']), np.float64(row['3-T03'])],
                            [np.float64(row['3-T10']), np.float64(row['3-T11']), np.float64(row['3-T12']), np.float64(row['3-T13'])], 
                            [np.float64(row['3-T20']), np.float64(row['3-T21']), np.float64(row['3-T22']), np.float64(row['3-T23'])],
                            [np.float64(row['3-T30']), np.float64(row['3-T31']), np.float64(row['3-T32']), np.float64(row['3-T33'])]])
            T04 = np.array([[np.float64(row['4-T00']), np.float64(row['4-T01']), np.float64(row['4-T02']), np.float64(row['4-T03'])],
                            [np.float64(row['4-T10']), np.float64(row['4-T11']), np.float64(row['4-T12']), np.float64(row['4-T13'])], 
                            [np.float64(row['4-T20']), np.float64(row['4-T21']), np.float64(row['4-T22']), np.float64(row['4-T23'])],
                            [np.float64(row['4-T30']), np.float64(row['4-T31']), np.float64(row['4-T32']), np.float64(row['4-T33'])]])
            T05 = np.array([[np.float64(row['5-T00']), np.float64(row['5-T01']), np.float64(row['5-T02']), np.float64(row['5-T03'])],
                            [np.float64(row['5-T10']), np.float64(row['5-T11']), np.float64(row['5-T12']), np.float64(row['5-T13'])], 
                            [np.float64(row['5-T20']), np.float64(row['5-T21']), np.float64(row['5-T22']), np.float64(row['5-T23'])],
                            [np.float64(row['5-T30']), np.float64(row['5-T31']), np.float64(row['5-T32']), np.float64(row['5-T33'])]])
            T06 = np.array([[np.float64(row['6-T00']), np.float64(row['6-T01']), np.float64(row['6-T02']), np.float64(row['6-T03'])],
                            [np.float64(row['6-T10']), np.float64(row['6-T11']), np.float64(row['6-T12']), np.float64(row['6-T13'])], 
                            [np.float64(row['6-T20']), np.float64(row['6-T21']), np.float64(row['6-T22']), np.float64(row['6-T23'])],
                            [np.float64(row['6-T30']), np.float64(row['6-T31']), np.float64(row['6-T32']), np.float64(row['6-T33'])]])
            T0EE = T06
            
            frame_transformations = np.array([T01, T02, T03, T04, T05, T06, T0EE])
            
            if(first_line):
                bot = Robot(joint_trafos=frame_transformations, use_double_precision=False)
                first_line = False
            else:
                ik_solutions = bot.IK(T0EE)
                avg_error, num_analytic, is_ls  = eval.evaluate_ik(bot, ik_solutions, T0EE, np.eye(3))

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

load_test_csv("/home/daniel/Documents/robot_1.csv")
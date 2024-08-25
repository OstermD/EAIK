import evaluate_ik as eval
from eaik.IK_Homogeneous import Robot
import numpy as np
import csv

def load_test_csv(paths):
    """
    Loads test robots from csv
    """
    errors = []
    for path in paths:
        bot = Robot(path, False)
        total_num_ls = 0
        error_sum = 0
        total_num_analytic = 0
        num_no_solution = 0
        with open(path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)

            T01 = None
            T02 = None
            T03 = None
            T04 = None
            T05 = None
            T06 = None

            for row in reader:
                q = np.array([np.float64(row['q1']), np.float64(row['q2']), np.float64(row['q3']),
                            np.float64(row['q4']), np.float64(row['q5']), np.float64(row['q6'])])
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

                frame_transformations = np.array([T01, T02, T03, T04, T05, T06])
                
                ik_solutions = bot.IK(T06)
                errors.append(eval.evaluate_ik(bot, ik_solutions, T06, np.eye(3)))
            
        with open("/home/daniel/Documents/EAIK/Testing/error_stability.csv", 'w', newline='') as f:
            writer = csv.writer(f)
            
            # Write the header
            writer.writerow(["Position_Error", "Orientation_Error"])
            
            # Write the rows 
            for err in errors:
                position_err = np.linalg.norm(err[0])
                orientation_err = np.linalg.norm(err[1])
                writer.writerow([f"{position_err:.17f}", f"{orientation_err:.17f}"])


load_test_csv(["/home/daniel/Documents/ostermeier_efficient_ik/data_daniel_float/robot_1.csv",
               "/home/daniel/Documents/ostermeier_efficient_ik/data_daniel_float/robot_2.csv",
               "/home/daniel/Documents/ostermeier_efficient_ik/data_daniel_float/robot_3.csv",
               "/home/daniel/Documents/ostermeier_efficient_ik/data_daniel_float/robot_4.csv",
               "/home/daniel/Documents/ostermeier_efficient_ik/data_daniel_float/robot_5.csv",
               "/home/daniel/Documents/ostermeier_efficient_ik/data_daniel_float/robot_6.csv",
               "/home/daniel/Documents/ostermeier_efficient_ik/data_daniel_float/robot_7.csv",
               "/home/daniel/Documents/ostermeier_efficient_ik/data_daniel_float/robot_8.csv",
               "/home/daniel/Documents/ostermeier_efficient_ik/data_daniel_float/robot_9.csv"])
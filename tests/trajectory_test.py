import numpy as np
import modern_robotics as mr

import sys
import os

# Dynamically add the project path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
from src.robot.youBot import youBot

current_dir = os.path.dirname(__file__)  
file_path = os.path.join(current_dir, "..", "inputs", "blist.csv")
blist = np.genfromtxt(file_path, delimiter=",", comments="#",filling_values=np.nan) 
blist = blist.T

file_path = os.path.join(current_dir, "..", "inputs", "M0e.csv")
M0e = np.genfromtxt(file_path, delimiter=",", comments="#",filling_values=np.nan)

file_path = os.path.join(current_dir, "..", "inputs", "Tbo.csv")
Tbo = np.genfromtxt(file_path, delimiter=",", comments="#",filling_values=np.nan)

file_path = os.path.join(current_dir, "..", "inputs", "Tsc_i.csv")
Tsc_initial = np.genfromtxt(file_path, delimiter=",", comments="#",filling_values=np.nan)

file_path = os.path.join(current_dir, "..", "inputs", "Tsc_f.csv")
Tsc_final = np.genfromtxt(file_path, delimiter=",", comments="#",filling_values=np.nan)

# chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state
Tsb0 = np.array([[1,0,0,0],
                [0,1,0,0],
                [0,0,1,0.0963],
                [0,0,0,1]]) #Intial Chassis configuration (b frame in s frame)
chassis_phi = 0
chassis_x = 0
chassis_y = 0
J1 = 0
J2 = 0 
J3 = 0
J4 = 0
J5 = 0 
W1 = 0
W2 = 0
W3 = 0
W4 = 0

robot = youBot()

Toe0 = mr.FKinBody(M0e,blist,[0,0,0,0,0])
Tse_initial = Tsb0@Tbo@Toe0
trajectory = robot.trajectory_generator(Tse_initial,Tsc_initial,Tsc_final)

np.savetxt(
    "outputs/trajectory_results.csv",
    trajectory,                   
    delimiter=",",
    fmt="%.6f",               
    header="chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4",
    comments="# "            
)
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

# chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state
Tsb = np.array([[1,0,0,0],
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
config = np.array([chassis_phi,chassis_x,chassis_y,J1,J2,J3,J4,J5,W1,W2,W3,W4,0])
dt = 0.01
N = 100
u_max = 12

u = np.array([10,10,10,10,1,1,1,1,1])
configs = np.array(config).reshape(1,-1)

for i in range(N):
    next_config = robot.next_state(config[0:12],u,dt,u_max)
    next_config = np.append(next_config,0)
    configs = np.concatenate([configs, next_config.reshape(1,-1)], axis=0)
    config = next_config

np.savetxt(
    "outputs/next_state_results.csv",
    configs,                   
    delimiter=",",
    fmt="%.6f",               
    header="chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4",
    comments="# "            
)

import numpy as np
import modern_robotics as mr
import sys
import os

# Dynamically add the project path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
from src.robot.youBot import youBot

current_dir = os.path.dirname(__file__)  
file_path = os.path.join(current_dir, "..", "inputs", "blist.csv")

file_path = os.path.join(current_dir, "..", "inputs", "Tsc_i.csv")
Tsc_initial = np.genfromtxt(file_path, delimiter=",", comments="#",filling_values=np.nan)

file_path = os.path.join(current_dir, "..", "inputs", "Tsc_f.csv")
Tsc_final = np.genfromtxt(file_path, delimiter=",", comments="#",filling_values=np.nan)

#----------------------------------------------

config_initial = np.zeros(13)
config_initial[3] = 0
config_initial[4] = -0.2
config_initial[5] = -0.2
config_initial[6] = -np.pi/2
config_initial[7] = 0
K = [np.zeros((6,6)),np.zeros((6,6))]

robot = youBot()
robot.u_max = 200
robot.limit = np.array([[-1,1],[-2,-0.05],[-2,-0.05],[-2,-0.05],[-1,1]])


Tsb_initial = robot.chassis_to_SE3(config_initial) 
Toe = mr.FKinBody(robot.M0e,robot.blist,config_initial[3:8])
Tse_initial = Tsb_initial@robot.Tbo@Toe

ref_traj = robot.trajectory_generator(Tse_initial,Tsc_initial,Tsc_final,k=1)

config_initial[2] = 0
result,error = robot.simulate_bot(ref_traj,K,config_initial)

np.savetxt(
    "outputs/simulation_test_reftraj.csv",
    ref_traj,                   
    delimiter=",",
    fmt="%.6f",
    comments="# "            
)

np.savetxt(
    "outputs/simulation_test_results.csv",
    result,                   
    delimiter=",",
    fmt="%.6f",               
    header="chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4",
    comments="# "            
)

np.savetxt(
    "outputs/simulation_test_error.csv",
    error,                   
    delimiter=",",
    fmt="%.6f",
    comments="# "            
)
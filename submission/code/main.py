import numpy as np
import modern_robotics as mr
import sys
import os
import matplotlib.pyplot as plt

""" plt.rcParams.update({
    "figure.dpi": 180,           # Higher-res figures for 4K
    "figure.figsize": (10, 6),   # Larger default figure size
    "font.size": 14,             # Bigger text everywhere
    "axes.labelsize": 14,
    "axes.titlesize": 16,
    "legend.fontsize": 12,
    "xtick.labelsize": 12,
    "ytick.labelsize": 12,
    "lines.linewidth": 2.0,      # Thicker lines
    "lines.markersize": 6,
}) """

# Dynamically add the project path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
from src.robot.youBot import youBot

current_dir = os.path.dirname(__file__)  

file_path = os.path.join(current_dir, "..", "inputs", "Tsc_i.csv")
Tsc_initial = np.genfromtxt(file_path, delimiter=",", comments="#",filling_values=np.nan)

file_path = os.path.join(current_dir, "..", "inputs", "Tsc_f.csv")
Tsc_final = np.genfromtxt(file_path, delimiter=",", comments="#",filling_values=np.nan)

#----------------------------------------------

Tsc_initial = np.array([[0,-1,0,-1],
                [1,0,0,0],
                [0,0,1,0.025],
                [0,0,0,1]])

Tsc_final = np.array([[1,0,0,0.5],
                [0,1,0,-0.5],
                [0,0,1,0.025],
                [0,0,0,1]])


config_initial = [30*np.pi/180,-0.5,-1,0,-0.15,-0.15,-np.pi/2,0,0,0,0,0,0]
joint_limits = np.array([[-100,100],[-2,-0.05],[-2,-0.05],[-2,-0.05],[-100,100]])
kR = 4 # rotational gain (for ωx, ωy, ωz)
kV = 4 # translational gain (for vx, vy, vz)
Ki = 0

Kp = np.diag([kR, kR, kR, kV, kV, kV])
Ki = np.diag([Ki,Ki,Ki,Ki,Ki,Ki])

K = [Kp,Ki]

robot = youBot()
robot.u_max = 50
robot.limit = joint_limits

Tse_initial = np.array([
    [0, 0, 1, 0],
    [0, 1, 0, 0],
    [-1, 0, 0, 0.5],
    [0, 0, 0, 1]
])

controller = "FFPI"
ref_traj = robot.trajectory_generator(Tse_initial,Tsc_initial,Tsc_final,k=1)
result,error = robot.simulate_bot(ref_traj,K,config_initial,controller)

idx = np.argmax(result[:, 12] == 1)
time = np.arange(len(error[:idx, 0])) * robot.dt * robot.k

#---------------------------------

np.savetxt(
    "outputs/main_reftraj.csv",
    ref_traj,                   
    delimiter=",",
    fmt="%.6f",
    comments="# "            
)

np.savetxt(
    "outputs/main_results.csv",
    result,                   
    delimiter=",",
    fmt="%.6f",               
    header="chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4",
    comments="# "            
)

np.savetxt(
    "outputs/main_error.csv",
    error,                   
    delimiter=",",
    fmt="%.6f",
    comments="# "            
)


fig, axs = plt.subplots(6, 1, figsize=(8, 10), sharex=True)
labels = [r'$\omega_x$', r'$\omega_y$', r'$\omega_z$', 
          r'$v_x$', r'$v_y$', r'$v_z$']

for i in range(6):
    axs[i].plot(time, error[:idx, i], linewidth=2)
    axs[i].set_ylabel(labels[i])
    axs[i].grid(True)

axs[-1].set_xlabel("Time [s]")
fig.suptitle(r"Elements of $X_{err}$ vs Time", fontsize=14)
plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.show()
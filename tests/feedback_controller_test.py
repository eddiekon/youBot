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

configuration = [0,0,0,0,0,0.2,-1.6,0]

Tsb0 = np.array([[1,0,0,0],
                [0,1,0,0],
                [0,0,1,0.0963],
                [0,0,0,1]])

Xd = np.array([
    [ 0,  0,  1, 0.5],
    [ 0,  1,  0, 0.0],
    [-1,  0,  0, 0.5],
    [ 0,  0,  0, 1.0]
])

Xd_next = np.array([
    [ 0,  0,  1, 0.6],
    [ 0,  1,  0, 0.0],
    [-1,  0,  0, 0.3],
    [ 0,  0,  0, 1.0]
])


X = np.array([
    [ 0.170,  0.0,  0.985, 0.387],
    [ 0.0,    1.0,  0.0,   0.0],
    [-0.985,  0.0,  0.170, 0.570],
    [ 0.0,    0.0,  0.0,   1.0]
])

K = [np.zeros((6,6)),np.zeros((6,6))]
dt = 0.01
robot = youBot()
robot.integral = np.zeros(6)
V = robot.feedback_control(X,Xd,Xd_next,K,dt)

print(V)

Jb = mr.JacobianBody(blist,configuration[3:])

F6 = np.concatenate((
    np.zeros((1, 4)),
    np.zeros((1, 4)),
    robot.F,
    np.zeros((1, 4))
), axis=0)

Jbase = mr.Adjoint(mr.TransInv(X)@Tsb0)@F6 


Je = np.hstack((Jbase, Jb))

controls = np.linalg.pinv(Je)@V
np.set_printoptions(precision=3, suppress=True)
print(controls)
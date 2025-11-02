Mobile Manipulation Capstone — youBot Pick-and-Place Simulation

This project simulates a KUKA youBot performing a pick-and-place task using the Modern Robotics framework by Lynch & Park. 
It satisfies the final programming requirement for the Mobile Manipulation Capstone.

Overview
--------
The robot starts at a given pose, moves toward a cube, picks it up, and places it at a target location — all while following 
a smooth, time-scaled trajectory. The simulation includes:
- Trajectory generation for seven motion segments (approach, grasp, lift, move, place, release).
- Feedback control with proportional and feedforward terms to minimize pose error.
- Kinematic simulation of chassis, arm, and wheel motion.
- Error visualization over time.

Main Files
----------
- youBot.py — Defines the youBot class, kinematics, trajectory generation, and control logic.
- main.py — Runs the simulation, saves results, and plots pose error (X_err) components.

Outputs
-------
After running main.py, the following files are generated in /outputs:
- main_reftraj.csv — Full reference trajectory.
- main_results.csv — Simulated robot configurations at each timestep.
- main_error.csv — Pose-error data used for plotting.
- A plot showing the six elements of X_err over time.

Run Instructions
----------------
1. Make sure modern_robotics.py is installed (pip install modern_robotics).
2. Run the simulation:
   python main.py
3. Results and plots will appear in the outputs folder.

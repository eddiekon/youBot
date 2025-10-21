import numpy as np
import modern_robotics as mr

class youBot:

    """
        Class for youBot #TODO
    """

    def __init__(self):
        pass

    def next_state(self,config,controls,dt,u_max):
        """
            Function: NextState
            Description:
                Kinematic simulator that calculates the next configuration of the robot
                dt time ahead based on inputed configurations and controls
            Inputs:
                config: A 12-vector representing the current configuration of the robot 
                    (3 variables for the chassis configuration, 5 variables for the arm configuration, 
                    and 4 variables for the wheel angles)
                controls: A 9-vector of controls indicating the wheel speeds u (4 variables) 
                    and the arm joint speeds θdot (5 variables)
                dt: Timestep
                u_max: A positive real value indicating the maximum angular speed of the arm joints 
                    and the wheels.
            Returns: A 12-vector representing the configuration of the robot time Δt later
        """
        pass
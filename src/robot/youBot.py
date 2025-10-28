import numpy as np
import modern_robotics as mr

class youBot:

    """
        Class for youBot #TODO
    """

    def __init__(self):
        """
            Initialization of hardcodded parameters
        """
        self.dt = 0.01 #Step time (s)
        self.r = 0.0475 #Radius of each wheel (m)
        self.l = 0.47/2 #Forward-backward distance between the wheels (m)
        self.w = 0.3/2 #Side-to-side distance between wheels (m)
        self.gamma = [-np.pi/4,np.pi/4,-np.pi/4,np.pi/4] #free sliding direction of each wheel (rad)
        self.wheel_locations = np.array([[self.l,self.w],
                                         [self.l,-self.w],
                                         [-self.l,-self.w],
                                         [-self.l,self.w]]) #Locations of the wheel in the chassis frame (m)
        gripper_d3 = 0.043 #The distance from the base of gripper the fingers to the end effector frame {e}
        cube_side = 0.05 #The length of the cube sides
        a = 5*np.pi/4#Rotation angle of the end effector frame around the cube's y axis  
        self.Tce_grasp = np.array([[np.cos(a),0,np.sin(a),np.cos(-np.pi/4)*(gripper_d3-np.sqrt(2*((cube_side/2)**2)))],
                                   [0,1,0,0],
                                   [-np.sin(a),0,np.cos(a),np.sin(-np.pi/4)*(gripper_d3-np.sqrt(2*((cube_side/2)**2)))],
                                   [0,0,0,1]]) #The end-effector's configuration relative to the cube when it is grasping the cube
        
        self.Tce_standoff = np.array([[np.cos(a),0,np.sin(a),0.3*np.cos(3*np.pi/4)],
                                      [0,1,0,0],
                                      [-np.sin(a),0,np.cos(a),0.3*np.sin(3*np.pi/4)],
                                      [0,0,0,1]]) #The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube. 
        self.v_max = 0.5 #Maximum linear velocity of the end-effector (m/s)
        self.omega_max = 4*np.pi #Maximum angular velocity of the end-effector (rad/s)
        self.grasp_time = 0.625 #Time it takes for gripper to transition from one state to another (sec)

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
        
        controls = np.clip(controls, -u_max, u_max) # Saturation 

        joint_angles = config[3:8]
        wheel_angles = config[8:]
        wheel_speeds = controls[0:4]
        joint_speeds = controls[4:]
        next_arm_joint_angles = joint_angles + joint_speeds*dt
        next_arm_wheel_angles = wheel_angles + wheel_speeds*dt

        dwheel = np.array(wheel_speeds*dt)

        #TODO: Obtain new chassis configuration
        wloc = self.wheel_locations
        h = np.empty((len(self.gamma),3))
        beta = np.zeros_like(self.gamma) #Angle of wheel relative to the chassis frame in the x,y plane.

        for i in range(len(self.gamma)):

            betas = np.array([[np.cos(beta[i]),np.sin(beta[i])],[-np.sin(beta[i]),np.cos(beta[i])]])
            wheel = np.array([[-wloc[i,1],1,0],[wloc[i,0],0,1]]) 
            h[i,:] = [1/self.r, np.tan(self.gamma[i])/self.r]@betas@wheel #Equation 13.5 from Modern Robotics by Kevin Lynch

        Vb = np.linalg.pinv(h)@dwheel #Body twist of chassis [wz,x,y]
        Vb6 = np.array([0,0,Vb[0],Vb[1],Vb[2],0])
        Tb0b1 = mr.VecTose3(Vb6) #Transformation matrix from old to new configuration for the chasssis

        wbz = Vb[0]
        vbx = Vb[1]
        vby = Vb[2]
        if wbz == 0: #If no rotation
            dqb = [0,vbx,vby]
        else:
            dqb = [wbz,
                   (vbx*np.sin(wbz)+vby*(np.cos(wbz)-1)),
                   (vby*np.sin(wbz)+vbx*(1-np.cos(wbz)))]

        phi0 = config[0]
        dq = np.array([[1,0,0],       #Transforming change of position to fixed frame
                       [0,np.cos(phi0),-np.sin(phi0)],
                       [0,np.sin(phi0),np.cos(phi0)]]) @ dqb 
        q0 = config[0:3]
        q1 = q0 + dq 

        new_config = np.concatenate([q1,next_arm_joint_angles,next_arm_wheel_angles])

        return new_config
    
    def trajectory_generator(self,Tse_initial,Tsc_initial,Tsc_final,Tce_grasp,Tce_standoff,k):
        
        
        def segment(initial,final):

            #Calculate distance and rotation angle to pick travel time that would fit the limits
            [Ri,pi] = mr.TransToRp(initial)
            [Rf,pf] = mr.TransToRp(final)            
            travel_distance = np.linalg.norm(pf-pi)
            Rrel = Rf @ Ri.T
            angle_rotation = (np.trace(Rrel) - 1.0) / 2.0 

            Tf = round(max(travel_distance/self.v_max,angle_rotation/self.omega_max)/self.dt)*self.dt #Time of segment (s)
            N = Tf * k / self.dt #Number of points 
            path = mr.CartesianTrajectory(initial,final,Tf,N)

            return path
        
        def grasp(configuration):

            configuration[-1] = 1 - configuration[-1] #Flip the gripper state
            N = round(self.grasp_time / self.dt) * self.dt
            rounded_up_N = int(-(-N // 1))
            return np.tile(configuration,N)
        
        #TODO: Build out each segment
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

        self.Tbo = np.array([
            [1, 0, 0, 0.1662],
            [0, 1, 0, 0.0],
            [0, 0, 1, 0.0026],
            [0, 0, 0, 1.0]
        ]) #Configuration of {o} (Base of arm) in relation to chassis frame

        self.blist = np.array([
            [0, 0, 1, 0, 0.033, 0],
            [0, -1, 0, -0.5076, 0, 0],
            [0, -1, 0, -0.3526, 0, 0],
            [0, -1, 0, -0.2176, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ]).T #When the arm is at its home configuration, the screw axes for the five joints are expressed in the end-effector frame {e} 

        self.M0e = np.array([
            [1, 0, 0, 0.033],
            [0, 1, 0, 0.0],
            [0, 0, 1, 0.6546],
            [0, 0, 0, 1.0]
        ]) #the end-effector frame {e} relative to the arm base frame {0} When the arm is at its home configuration

        self.u_max = 10 #Maximum speed
        self.z = 0.0963 #Height of the chassis body frame {b}
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
        a = -5*np.pi/4#Rotation angle of the end effector frame around the cube's y axis  

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

        self.F = (self.r / 4) * np.array([
            [-1 / (self.l + self.w),  1 / (self.l + self.w),  1 / (self.l + self.w), -1 / (self.l + self.w)],
            [ 1,                      1,                      1,                      1                     ],
            [-1,                      1,                     -1,                      1                     ]
        ]) # wheel configuration matrix for a omnidirectional robot

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
        Tb0b1 = mr.VecTose3(Vb6) #Transformation matrix from old to new configuration for the chassis

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
    
    def trajectory_generator(self,Tse_initial,Tsc_initial,Tsc_final,Tce_grasp=None,Tce_standoff=None,k=1):
        """
            Function: trajectory_generator
            Description:
                Generates the full end-effector trajectory for a pick-and-place operation
                based on the initial robot and cube configurations.

                The trajectory consists of seven sequential motion segments:
                    1. Move from the robot's initial end-effector pose to the standoff above the cube.
                    2. Move straight down to the cube grasp pose.
                    3. Close the gripper to grasp the cube.
                    4. Move straight back up to the standoff pose.
                    5. Move to the standoff pose above the cube's final position.
                    6. Move straight down to the cube's final placement pose.
                    7. Open the gripper to release the cube.

                Each motion segment is generated using a helper function that ensures
                the trajectory obeys maximum velocity and angular rate limits.
                The full trajectory is formed by concatenating the segments in order.

            Inputs:
                Tse_initial: Initial end-effector configuration (4x4 homogeneous transform)
                Tsc_initial: Initial cube configuration (4x4 homogeneous transform)
                Tsc_final: Final cube configuration (4x4 homogeneous transform)
                Tce_grasp: End-effector configuration relative to cube during grasp
                Tce_standoff: End-effector configuration relative to cube during standoff
                k: Number of trajectory reference configurations per 0.01 seconds (frequency scaling factor)

            Returns:
                configuration: A matrix of all trajectory configurations where each row represents
                            a single time step consisting of:
                            - 9 rotation matrix elements (row-major order)
                            - 3 position elements
                            - 1 gripper state (0 = open, 1 = closed)
        """
        
        def segment(initial,final,gripperstate):
            """
                Helper Function: segment
                Description:
                    Generates a smooth Cartesian trajectory between two 4x4 configurations
                    (initial and final) using quintic time scaling for smooth motion.

                    Computes the travel time based on maximum allowed linear and angular speeds,
                    then samples the motion at discrete time intervals.

                Inputs:
                    initial: Starting 4x4 homogeneous transformation
                    final: Ending 4x4 homogeneous transformation
                    gripperstate: Scalar indicating gripper state (0 = open, 1 = closed)

                Returns:
                    path: An array where each row contains:
                        - Flattened 3x3 rotation matrix (9 elements)
                        - Translation vector (3 elements)
                        - Gripper state (1 element)
            """
            #Calculate distance and rotation angle to pick travel time that would fit the limits
            [Ri,pi] = mr.TransToRp(initial)
            [Rf,pf] = mr.TransToRp(final)            
            travel_distance = np.linalg.norm(pf-pi)
            Rrel = Rf @ Ri.T
            angle_rotation = (np.trace(Rrel) - 1.0) / 2.0 

            Tf = round(max(travel_distance/self.v_max,angle_rotation/self.omega_max)/self.dt)*self.dt #Time of segment (s)
            N = Tf * k / self.dt #Number of points 
            result = mr.CartesianTrajectory(initial,final,Tf,N,5)

            path = np.array([
            np.concatenate((
                m[:3, :3].flatten(),   # top-left 3×3 block
                m[:3, 3],              # first 3 elements of last column
                np.array([gripperstate])))  # wrap scalar into array
                for m in result])

            return path
        
        def grasp(configuration):
            """
                Helper Function: grasp
                Description:
                    Toggles the gripper state (open ↔ closed) and holds it constant
                    for the duration required to complete the grasp or release action.

                Inputs:
                    configuration: A single configuration row from the trajectory
                                representing the current end-effector pose and gripper state.

                Returns:
                    result: A repeated set of the input configuration (N×13 array)
                            held constant for the full grasp or release time interval.
            """
            
            configuration[-1] = 1 - configuration[-1] #Flip the gripper state
            N = int(np.ceil(self.grasp_time / self.dt))
            result = np.tile(configuration,(N,1))
            return result
        

        if Tce_grasp is None:
            Tce_grasp = self.Tce_grasp
        if Tce_standoff is None:
            Tce_standoff = self.Tce_standoff
        self.k = k
        # 1.Move to grasp standoff
        configuration = segment(Tse_initial,Tsc_initial@Tce_standoff,0)
        # 2.Move to grasp
        configuration = np.concatenate((configuration,segment(Tsc_initial@Tce_standoff,Tsc_initial@Tce_grasp,configuration[-1,-1])))
        # 3.Grasp
        configuration = np.concatenate((configuration,grasp(configuration[-1, :])))
        # 4.Move back up to grasp standoff
        configuration = np.concatenate((configuration,segment(Tsc_initial@Tce_grasp,Tsc_initial@Tce_standoff,configuration[-1,-1])))
        # 5.Move to goal configuration standoff
        configuration = np.concatenate((configuration,segment(Tsc_initial@Tce_standoff,Tsc_final@Tce_standoff,configuration[-1,-1])))
        # 6.Move to goal configuration
        configuration = np.concatenate((configuration,segment(Tsc_final@Tce_standoff,Tsc_final@Tce_grasp,configuration[-1,-1])))
        # 7.Release the block
        configuration = np.concatenate((configuration,grasp(configuration[-1, :])))

        return configuration
    
    def feedback_control(self,X,Xd,Xd1,K,dt):

        [Kp,Ki] = K
        Xerr = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(Xd) @ X))
        self.integral = self.integral + Xerr*dt
        Vdmat = (1/dt)*mr.MatrixLog6(mr.TransInv(Xd)@Xd1)
        Vd = mr.se3ToVec(Vdmat)
        Ve = mr.Adjoint(mr.TransInv(X)@Xd)@Vd + Kp@Xerr + Ki@self.integral #PID Controller, end effector twist.
        
        return Ve
    
    def simulate_bot(self,trajectory,K,config_initial):
        
        k = self.k
        N = len(trajectory)
        self.integral = 0
        #Loop through generated trajectory step by step

        config_list = np.array([config_initial]) #List of configurations for results
        Xerr_list = np.empty((1,6)) #List of errors at every step
        configuration = config_initial

        for i in range(N-1):
            
            #Get the end effector configuration
            Tsb = self.chassis_to_SE3(configuration) 
            Toe = mr.FKinBody(self.M0e,self.blist,configuration[3:8])
            X = Tsb@self.Tbo@Toe

            #Use the controller at this time step for end effector twist needed
            Xd = self.chassis_to_SE3(trajectory[i,:]) #Reference configuration 
            Xdnext = self.chassis_to_SE3(trajectory[i+1,:]) #Next reference configuration
            Xerr = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(Xd) @ X)) #Error at this step
            Ve = self.feedback_control(X,Xd,Xdnext,K,self.dt) #Get end effector twist needed {e}

            #Calculate Jacobian
            Jb = mr.JacobianBody(self.blist,configuration[3:8])
            F6 = np.concatenate((
                np.zeros((1, 4)),
                np.zeros((1, 4)),
                self.F,
                np.zeros((1, 4))
            ), axis=0)
            Jbase = mr.Adjoint(mr.TransInv(X)@Tsb)@F6 

            np.set_printoptions(precision=3, suppress=True)
            Je = np.hstack((Jbase, Jb))

            #Calculate controls required for the twist that is needed
            controls = np.linalg.pinv(Je,0.01)@Ve

            #Update configuration
            configuration = self.next_state(configuration[:12],controls,self.dt,self.u_max)
            configuration = np.append(configuration,trajectory[i,12])

            if i%k == 0: #Save Configuration and Xerr for results
                Xerr_list = np.vstack((Xerr_list,Xerr))
                config_list = np.vstack((config_list,configuration))

        return config_list,Xerr_list
    
    def chassis_to_SE3(self,configuration):

            phi, x, y = configuration[:3]

            R = np.array([
                [np.cos(phi), -np.sin(phi), 0],
                [np.sin(phi),  np.cos(phi), 0],
                [0,            0,           1]
            ])

            p = np.array([[x], [y], [self.z]])  # position on the ground plane

            T = np.block([
                [R, p],
                [np.zeros((1, 3)), 1]
            ])
            return T

                

        
        
        

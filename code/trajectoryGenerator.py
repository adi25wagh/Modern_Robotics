import numpy as np
import modern_robotics as mr

import os.path
import csv

class Cube_Configuration:
    """
    Storage object to save configurations of the cube and the relate configurations required from the robot arm.
    This object must be constructed prior to simulation / trajectory generation
    """
    def __init__(self, 
                T_start: np.array = np.array([
                                            [1, 0, 0, 1],
                                            [0, 1, 0, 0],
                                            [0, 0, 1, 0.025],
                                            [0, 0, 0, 1]
                                            ]), 
                T_final: np.array = np.array([
                                    [0, 1, 0, 0],
                                    [-1, 0, 0, -1],
                                    [0, 0, 1, 0.025],
                                    [0, 0, 0, 1]
                                    ]),
                T_standoff: np.array = np.array([
                                                [-0.7071, 0 , 0.7071, 0],
                                                [0,1,0,0],
                                                [-0.7071, 0, -0.7071, 0.1],
                                                [0,0,0,1]]),
                T_grasp: np.array = np.array([
                                            [-0.7071, 0 , 0.7071, 0],
                                            [0,1,0,0],
                                            [-0.7071, 0, -0.7071, 0],
                                            [0,0,0,1]])):
        self.T_sc_init = T_start
        self.T_sc_final = T_final
        self.T_standoff = T_standoff
        self.T_grasp = T_grasp

def trajSolver(solverType):
    solver = mr.CartesianTrajectory
    if solverType == "Screw":
        solver = mr.ScrewTrajectory
    return solver

class trajectoryGenerator:
    """
    The class generates a specific trajectory only for:
    1. Pick and place operation as for the project.
    
    Generalized additional calculators should allow for:
    1. Simulation of different trajectories in the same format
    """
    def __init__(self,
                 # NEEDED INPUT
                 T_se_init: np.array = np.array([
                                                [0, 0, 1, 0.2],
                                                [0, 1, 0, 0.2],
                                                [-1, 0, 0, 0.5],
                                                [0, 0, 0, 1],
                                            ]), # Specified initial location of the end effector at init
                
                 # ADDITIONAL INPUTS
                 k: int = 10, # Number of configurations per timestep
                 dt: float = 0.01, # Time-step for simulation
                 solver_type: str = "Cartesian",
                 methodval: int = 5):

        # Get needed solver from MR Library
        self.trajectorySolver = trajSolver(solverType=solver_type)
        # Cube configuration
        cube_config = Cube_Configuration()
        # Design values
        self.k = k
        self.dt = dt
        self.methodval = methodval

        """
        SOLVE FOR Sections IN THE TRAJECTORY
        """
        self.numSegments = 9
        T_start = np.empty((self.numSegments, 4, 4))
        T_end = np.empty((self.numSegments, 4, 4))
        grip_flag = np.empty(self.numSegments)

        # Move from initial location to standoff
        T_start[0] = T_se_init
        T_end[0] = cube_config.T_sc_init.dot(cube_config.T_standoff)
        grip_flag[0] = 0
        
        # Move from standoff to grasp
        T_start[1] = cube_config.T_sc_init.dot(cube_config.T_standoff)
        T_end[1] = cube_config.T_sc_init.dot(cube_config.T_grasp)
        grip_flag[1] = 0
        
        # Hold at grasp
        T_start[2] = T_end[1]
        T_end[2] = T_end[1]
        grip_flag[2] = 1

        # Move from grasp to standoff
        T_start[3] = T_end[1]
        T_end[3] = cube_config.T_sc_init.dot(cube_config.T_standoff)
        grip_flag[3] = 1

        # Move from standoff to standoff2
        T_start[4] = cube_config.T_sc_init.dot(cube_config.T_standoff)
        T_end[4] = cube_config.T_sc_final.dot(cube_config.T_standoff)
        grip_flag[4] = 1

        # Move from standoff2 to grasp
        T_start[5] = cube_config.T_sc_final.dot(cube_config.T_standoff)
        T_end[5] = cube_config.T_sc_final.dot(cube_config.T_grasp)
        grip_flag[5] = 1

        # Hold at grasp
        T_start[6] = T_end[5]
        T_end[6] = T_end[5]
        grip_flag[6] = 0

        # Move to standoff2
        T_start[7] = T_end[5]
        T_end[7] = cube_config.T_sc_final.dot(cube_config.T_standoff)
        grip_flag[7] = 0

        # Move back to init
        T_start[8] = cube_config.T_sc_final.dot(cube_config.T_standoff)
        T_end[8] = T_se_init
        grip_flag[8] = 0

        timeSegments = np.ones(self.numSegments)
        for tidx in range(self.numSegments):
            changeDist, maxChangeAng = self.computeChange(T_start[tidx], T_end[tidx])
            time_factor = np.max((changeDist / 0.2, maxChangeAng / 0.2))
            timeSegments[tidx] = np.round(np.max((time_factor, 0.8)), 2)

        # Store Variables
        self.timeSegments = timeSegments
        self.T_start = T_start
        self.T_end = T_end
        self.grip_flag = grip_flag

        # Allocate Space for trajectory
        self.Trajectory = np.empty((int(np.sum(timeSegments) * k / dt), 4, 4))
        self.gripperValue = np.zeros(int(np.sum(timeSegments) * k / dt))
    
    def solve(self):
        timeStamp = 0
        for idx in range(self.numSegments):

            # Values analogous to MR library
            Xstart = self.T_start[idx]
            Xend = self.T_end[idx]
            Tf = self.timeSegments[idx]
            N = int(self.timeSegments[idx] * self.k / self.dt)

            # Append values to trajectory
            self.Trajectory[timeStamp:timeStamp+N, :, :] = self.trajectorySolver(Xstart, Xend, Tf, N, self.methodval)
            if self.grip_flag[idx]:
                self.gripperValue[timeStamp:timeStamp+N] = 1
            
            # Add timestamp
            timeStamp += N

    @staticmethod
    def computeChange(T1, T2):
        T = mr.TransInv(T1).dot(T2)

        # Angular Change
        phi = 0.0
        if np.isclose(T[2, 0], -1.0, rtol=1E-5, atol=1E-7):
            theta = np.pi / 2
            psi = np.arctan2(T[0, 1], T[0, 2])
        elif np.isclose(T[2, 0], 1.0, rtol=1E-5, atol=1E-7):
            theta = -np.pi / 2
            psi = np.arctan2(-T[0, 1], -T[0, 2])
        else:
            theta = -np.arcsin(T[2, 0])
            cTheta = np.cos(theta)
            psi = np.arctan2(T[2, 1] / cTheta, T[2, 2] / cTheta)
            phi = np.arctan2(T[1, 0] / cTheta, T[0, 0] / cTheta)

        # Linear Distance
        q = T[0:3, -1]
        return np.linalg.norm(q), np.max((theta, phi, psi))
    
    def tolist(self):
        outlist = []
        for idx in range(len(self.gripperValue)):
            traj = self.Trajectory[idx, :, :]
            grip = self.gripperValue[idx]
            intlist = [traj[0, 0], traj[0, 1], traj[0, 2],
                       traj[1, 0], traj[1, 1], traj[1, 2],
                       traj[2, 0], traj[2, 1], traj[2, 2],
                       traj[0, 3], traj[1, 3], traj[2, 3],
                       grip]
            outlist.append(intlist)
        return outlist
    
    def toTransformations(self):
        outlist = []
        for idx in range(self.Trajectory.shape[0]):
            traj = self.Trajectory[idx,:,:]
            outlist.append(traj)
        
        return outlist, self.gripperValue

def execute_test():
    T_start = np.array([
        [0, 0, 1, 0.2],
        [0, 1, 0, 0.2],
        [-1, 0, 0, 0.5],
        [0, 0, 0, 1],
    ])
    
    traj_obj = trajectoryGenerator(T_se_init=T_start, solver_type="Cartesian", k=10, dt=0.01, methodval=5)
    traj_obj.solve()
    traj = traj_obj.tolist()
    itr = 1
    for tr in traj:
        print(f"Current step {itr}, traj = {tr}")
        itr += 1

    write_test_file(traj)

def write_test_file(listval):
    pathfolder = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(pathfolder, 'test_trajectory2.csv'), 'w', newline='') as csv_file:
        obj = csv.writer(csv_file)
        for row in listval:
            obj.writerow(row)

if __name__ == "__main__":
    execute_test()
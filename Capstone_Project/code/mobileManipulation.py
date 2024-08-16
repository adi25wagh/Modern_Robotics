import numpy as np
from trajectoryGenerator import trajectoryGenerator
from nextState import nextState
from controller import controller

import matplotlib.pyplot as plt
import csv
import os.path

def mobileManipulation():
    # Initialize
    segmentsPerTimeStep = 5
    timeStep = 0.01

    # Setup constants
    FFon = False # 1
    KP = 5.5 # 5.5
    KI = 3  # 3

    trajInit = np.array([
        [0, 0, 1, 0.3],
        [0, 1, 0, 0.1],
        [-1, 0, 0, 0.5],
        [0, 0, 0, 1],
        ])
    
    # Gather previously completed parts.
    trajGen = trajectoryGenerator(T_se_init= trajInit, solver_type = "Cartesian", k = segmentsPerTimeStep, dt = timeStep, methodval = 5)
    eulerSolver = nextState(delta_t = timeStep / segmentsPerTimeStep, omega_max = [40,40,40,40,40, 60, 60, 60, 60])
    feedbackCont = controller(ff = FFon, kp = KP, ki = KI, dt = timeStep, k = segmentsPerTimeStep)

    # compute Trajectory
    trajGen.solve()
    trajectory, gripperState = trajGen.toTransformations()
    nTimeStamps = len(trajectory)

    # initialize starting state of robot
    robotState = np.array([0.5,-0.4,0.4, 0.0, 0.5, -0.8, -0.9, 1,0,0,0,0,0])
    robotState[12] = gripperState[0] # initialize gripper state
    robotTraj = [] # Variable to store the trajectory of the robot
    stateError = []
    rmsError = []
    integralError = []
    idx2 = 1 # additional index to store values at instances.
    for idx in range(nTimeStamps-1):
        # compute inputs to controller
        Xd = trajectory[idx]
        Xdn = trajectory[idx+1]
        grip = gripperState[idx]
        
        # compute control inputs
        control_inputs, Xerr, Xinterr= feedbackCont.computeControlInput(robotState[0:8], Xd, Xdn)

        # Update storage lists
        if idx % segmentsPerTimeStep == 0:
            idx2 += 1
            robotTraj.append(robotState.tolist())
            stateError.append(Xerr.tolist())
            rmsError.append(np.linalg.norm(Xerr))
            integralError.append(Xinterr.tolist())

        # Propagate state
        robotState[0:12] = eulerSolver.step(robotState[0:12], control_inputs)
        robotState[12] = grip
        
    # Print rms error to allow for quick testing.
    print(f'RMS Error: {np.mean(rmsError)}')

    # Plot trajectory error
    stateError = np.array(stateError)

    fig,ax = plt.subplots(1,1)
    ax.plot(np.arange(1,idx2,dtype=float)*timeStep, stateError[:,0], label= "$\hat{v}_x$")
    ax.plot(np.arange(1,idx2,dtype=float)*timeStep, stateError[:,1], label= "$\hat{v}_y$")
    ax.plot(np.arange(1,idx2,dtype=float)*timeStep, stateError[:,2], label= "$\hat{v}_z$")
    ax.plot(np.arange(1,idx2,dtype=float)*timeStep, stateError[:,3],label= "$\hat{\omega}_x$")
    ax.plot(np.arange(1,idx2,dtype=float)*timeStep, stateError[:,4],label= "$\hat{\omega}_y$")
    ax.plot(np.arange(1,idx2,dtype=float)*timeStep, stateError[:,5],label= "$\hat{\omega}_z$")
   
    leg = plt.legend(loc='best')
    fig.suptitle("$X_{err}$")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("Error")
    plt.show()

    write_test_file(robotTraj, 'overshootStates.csv')
    write_test_file(stateError, 'overshootError.csv')
    
def write_test_file(trajectory, name):
    """ Write resulting path list to file """
    pathfolder = os.path.join(os.path.split(__file__)[0])
    with open(os.path.join(pathfolder,name), 'w') as csv_file:
        csv_vals = csv.writer(csv_file)
        for rows in trajectory:
            csv_vals.writerow(rows)

if __name__=="__main__":
    mobileManipulation()
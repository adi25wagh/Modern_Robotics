import numpy as np

import os.path
import csv

class nextState:
    def __init__(self,
                 delta_t: float = 0.01, # simulation time step
                 omega_max: list = [20,20,20,20,20, np.Inf, np.Inf, np.Inf, np.Inf] # Max speed for joints
                 ):
        self.dt = delta_t
        self.w_max = omega_max
        # Matrix related to odometry
        l = 0.47/2
        w = 0.3/2
        r = 0.0475
        self.H_mat = (r/4) * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],[1,1,1,1],[-1,1,-1,1]])
    
    def step(self,
             current_state, # current state representated as [[phi, x, y, J1, J2, J3, J4, J5, W1, W2, W3, W4]].T
             control_inputs # The inputs to change state [[J1,J2,J3,J4,J5,u1,u2,u3,u4]].T
             ):
        
        control_inputs = self.limitJointVelocity(control_inputs)
        
        next_state = np.zeros((12,))

        # Changes related to [phi, x, y]
        next_state[0:3] = self.wheelOdometry(current_state[0:3], control_inputs[5:9])
        
        # Changes realted to [W1,W2,W3,W4,J1,J2,J3,J4,J5]
        next_state[3:] = self.updateRobotAngles(current_state[3:], control_inputs[0:9])
        
        return next_state

    def limitJointVelocity(self, theta_dot):
        # Check each joint for velocity limit.
        for idx in range(theta_dot.shape[0]):
            if np.abs(theta_dot[idx]) > self.w_max[idx]:
                theta_dot[idx] = self.w_max[idx]
        return theta_dot
    
    def wheelOdometry(self, current_state, theta_dot):
        V_b = np.dot(self.H_mat, theta_dot*self.dt)
        wbz,vbx,vby = V_b
        phi = current_state[0]

        if wbz < 1E-3:
            dqb = np.array([0,vbx,vby])
        else:
            dqb = np.array([wbz, 
                            vbx*np.sin(wbz)+vby*(np.cos(wbz)-1)/wbz,
                            vby*np.sin(wbz)+vbx*(1-np.cos(wbz))/wbz])
            
        # Update matrix
        T_s = np.array([[1, 0, 0],
                     [0, np.cos(phi), -np.sin(phi)],
                     [0, np.sin(phi), np.cos(phi)]])
        
        deltaqsb = np.dot(T_s, dqb)
        next_state = current_state + deltaqsb 
        return next_state
    
    def updateRobotAngles(self, current_state, control_inputs):
        assert (current_state.shape[0] == 9)
        assert (control_inputs.shape[0] == 9)
        
        # Update thetas and wheels
        return current_state + (self.dt * control_inputs) 

def execute_test():
    control_inputs = np.array([0.0,0.0,0.0,0.0,0.0,10,-10,-10,10])
    current_state = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
    traj = current_state.reshape((1,12)).tolist()
    State_Calc = nextState(delta_t = 0.01, omega_max = 5)
    for itr in range(1,1000):
        next_step = State_Calc.step(current_state, control_inputs)
        traj.append(next_step.tolist())
    
    write_test_file(traj)

def write_test_file(trajectory):
    """ Write resulting path list to file """
    pathfolder = os.path.join(os.path.split(__file__)[0])
    with open(os.path.join(pathfolder,'test_trajectory1.csv'), 'w') as csv_file:
        csv_vals = csv.writer(csv_file)
        for rows in trajectory:
            csv_vals.writerow(rows)

if __name__ == "__main__":
    execute_test()



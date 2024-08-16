import numpy as np
import modern_robotics as mr

class controller:
    def __init__(self, ff: bool = True, kp: float = 0.0, ki: float = 0.0,  dt: float = 0.01, k: float = 10):
        
        # controller simulation time (faster than actual update time step.)
        self.delta_t = dt / k

        # Control input matrices
        self.Ki = ki*np.eye(6)
        self.Kp = kp*np.eye(6)
        self.ff = np.zeros((6,6))
        if ff:
            self.ff = np.eye(6)
    
        # Matrix to map twist vector to wheen inputs
        r = 0.0475
        l = 0.235
        w = 0.15
        self.F = r/4 * np.array([[0,0,0,0],[0,0,0,0],[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)],[1,1,1,1],[-1,1,-1,1],[0,0,0,0]])

        # transformation from {b} to {s}
        self.T_sb = lambda q: np.array([
            [np.cos(q[0]), -np.sin(q[0]), 0, q[1]],
            [np.sin(q[0]),  np.cos(q[0]), 0, q[2]],
            [0,             0,            1, 0.0963],
            [0,             0,            0, 1]])
       
        # static transformation from {e}/{0} to {b}
        self.Tb0 = np.array([[ 1, 0, 0, 0.1662],
				    [ 0, 1, 0,   0],
				    [ 0, 0, 1, 0.0026],
				    [ 0, 0, 0,   1]])
        
        # Mlist for robot arm (for FK)
        self.Mlist = np.array([[ 1, 0, 0, 0.033],
				    [ 0, 1, 0,   0],
				    [ 0, 0, 1, 0.6546],
				    [ 0, 0, 0,   1]])
        
        # Blist for robot arm (for FK)
        self.Blist = np.array([[0, 0, 1,   0, 0.033, 0],
                      [0,-1, 0,-0.5076,  0, 0],
                      [0,-1, 0,-0.3526,  0, 0],
                      [0,-1, 0,-0.2176,  0, 0],
                      [0, 0, 1,   0,     0, 0]]).T

        # Integral Error storage container.
        self.integral_error = np.zeros((6,))

    def computeControlInput(self, current_configuration, Xd, Xdn):
        # Extract configurations
        assert (current_configuration.shape[0] == 8)
        theta = current_configuration[3:8] # Extract arm angles from current state
        location = current_configuration[0:3] # extract [phi, x ,y] from onfiguration
        
        # Convert current Configuration to X
        Tsb = self.T_sb(location) # compute transformation from {s} to {b}
        T0e = mr.FKinBody(self.Mlist, self.Blist, theta) # Compute transformation from {0} to {e} using FK
        X = np.dot(Tsb, np.dot(self.Tb0, T0e)) # compute current tranformation matrix

        # Compute desired twist
        Vd = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Xd), (1/self.delta_t)*Xdn))) # Desired Motion transformed to Vec
        Adj = mr.Adjoint(np.dot(mr.TransInv(X), Xd)) # Transformation to end effector frame
        Xerr = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X), Xd))) # Compute current error state
        self.integral_error = self.integral_error + Xerr * self.delta_t # Increment integral errro upto current state

        # Desired twist is FF + PI
        V = ( np.dot(self.ff, np.dot(Adj, Vd)) # Feedforward
            + np.dot(self.Kp,Xerr) # Proportional correction
            + np.dot(self.Ki,self.integral_error) )# Integral correction


        J_arm = mr.JacobianBody(self.Blist, theta) # compute Jacobina for arm
        J_base = mr.Adjoint(np.dot(mr.TransInv(T0e), mr.TransInv(self.Tb0))).dot(self.F) # compute jacobian for base
        Jacobian = np.concatenate((J_base,J_arm), axis=1)
        Jacobian = self.testJointLimits(Jacobian, V, theta) # apply limits for body jacobian
        control_inputs = np.linalg.lstsq(Jacobian, V,rcond=1E-3)[0] # Here in format [w1,w2,w3,w4,j1,j2,j3,j4,j5]
        control_inputs = np.r_[control_inputs[4:9], control_inputs[0:4]]
        # Reshape control Inputs
        return control_inputs, Xerr, self.integral_error
    
    def testJointLimits(self, J, V, theta):
        # Predeclare limits [J1,J2,J3,J4,J5]
        limits = [np.Inf, 1.18, 2.16, 1.8, np.Inf]
        # Compute change
        u = np.linalg.lstsq(J, V, rcond=1E-3)[0]
        theta_tplus = theta + u[4:] * self.delta_t
        # Apply limits
        for idx in range(0,theta_tplus.shape[0]):
            if np.abs(theta_tplus[idx]) > limits[idx]:
                J[:,4+idx] = 0.0
        # Return J
        return J
    
def execute_test():
    # test to verify functionality.
    config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
    Xd = np.array([[ 0, 0, 1, 0.5],
			   [ 0, 1, 0,   0],
			   [-1, 0, 0, 0.5],
			   [ 0, 0, 0,   1]])

    Xd_next = np.array([[0, 0, 1,   0.6],
					[0, 1, 0,     0],
					[-1, 0, 1, 0.3],
					[0, 0, 0,     1]])

    # X = np.array([[ 0.170, 0, 0.985, 0.387],
	# 		[0, 1, 0,   0],
	# 		[-0.985, 0, 0.170, 0.570],
	# 		[ 0, 0, 0,     1]])
    
    cont = controller(dt = 0.01, ki = 0, kp = 0, kd=0, k = 1)
    controlinputs, error, int_error, diffErr = cont.computeControlInput(config, Xd, Xd_next)

    print(f'Control Inputs are: {np.round(controlinputs, 2)} \n')
    print(f'Error is: {np.round(error,2)} \n')
    print(f'Integral Error is: {np.round(int_error,2)}. \n')

if __name__ == '__main__':
    execute_test()
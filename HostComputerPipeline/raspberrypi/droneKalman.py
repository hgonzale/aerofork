import numpy as np
import math
import pdb

class DroneEKF():
    '''
    An EKF for mouse tracking
    '''

    def __init__(self):

        #12 states
        n=12
        #13 measurements
        m=13

        #prediction noise
        pval=0.1

        #process noise
        qval=1e-4

        #measurement noise
        rval=0.1

        #initial control vector
        u=(0,0,0,0)

        #gravitational constant
        self.g=9.81
        #moment of inertia in x frame
        self.momentXX=0.00536
        #moment of inertia in y frame
        self.momentYY=0.00536
        #coefficient of force for motors
        self.Kf=1/400
        #moment of inertia for blades
        self.Ki=0.001
        #length of arms for quadrotor
        self.Lr=.292
        #mass of quadrotor
        self.mass=0.794
        #drag coefficient of air
        self.b=0.5

        self.timeDelta = 0.1

        '''
        Creates a KF object with n states, m observables, and specified values for 
        prediction noise covariance pval, process noise covariance qval, and 
        measurement noise covariance rval.
        '''

        # No previous prediction noise covariance
        self.P_pre = None

        # Current state is zero, with diagonal noise covariance matrix
        self.x = np.zeros((n,1))
        self.P_post = np.eye(n) * pval

        # Get state transition and measurement Jacobians from implementing class
        self.F = self.getF(self.x,u)
        self.H = self.getH(self.x)

        # Set up covariance matrices for process noise and measurement noise
        self.Q = np.eye(n) * qval
        self.R = np.eye(m) * rval
 
        # Identity matrix will be usefel later
        self.I = np.eye(n)

    def step(self, z, u):
        '''
        Runs one step of the EKF on observations z, where z is a tuple of length M.
        Returns a NumPy array representing the updated state.
        '''

        # Predict ----------------------------------------------------

        # $\hat{x}_k = f(\hat{x}_{k-1})$
        self.x = self.f(self.x,u)

        # $P_k = F_{k-1} P_{k-1} F^T_{k-1} + Q_{k-1}$
        self.P_pre = self.F * self.P_post * self.F.T + self.Q

        # Update -----------------------------------------------------

        # $G_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1}$
        G = np.dot(np.dot(self.P_pre,self.H.T), np.linalg.inv(np.dot(np.dot(self.H,self.P_pre),self.H.T) + self.R))

        # $\hat{x}_k = \hat{x_k} + G_k(z_k - h(\hat{x}_k))$
        
        self.x += np.dot(G, (np.array(z) - self.h(self.x).T).T)

        # $P_k = (I - G_k H_k) P_k$
        self.P_post = np.dot(self.I - np.dot(G, self.H), self.P_pre)

        # return self.x.asarray()
        return self.x


    def updateTimeDelta(self,input):
        self.timeDelta=input

    def f(self, x,u):
        #qx
        retVal=np.array(x[3])
        #qy
        retVal=np.vstack([retVal,x[4]])
        #qz
        retVal=np.vstack([retVal,x[5]])
        #vx
        retVal=np.vstack([retVal,-self.b/self.mass*x[3]+1/self.mass*math.sin(x[7])*self.Kf*(sum(u))])
        #vy
        retVal=np.vstack([retVal,-self.b/self.mass*x[4]+1/self.mass*math.sin(x[6])*math.cos(x[7])*self.Kf*(sum(u))])
        #vz
        retVal=np.vstack([retVal,-self.b/self.mass*x[5]-self.g+1/self.mass*math.cos(x[6])*math.cos(x[7])*self.Kf*(sum(u))])
        #qtheta
        retVal=np.vstack([retVal,x[9]])
        #qphi
        retVal=np.vstack([retVal,x[10]])
        #qpsi
        retVal=np.vstack([retVal,x[11]])
        #vtheta
        retVal=np.vstack([retVal,-x[10]*x[11]+1/self.momentXX*self.Lr*self.Kf*(u[1]-u[3])])
        #vphi
        retVal=np.vstack([retVal,x[9]*x[11]+1/self.momentYY*self.Lr*self.Kf*(-u[0]+u[2])])
        #vpsi
        retVal=np.vstack([retVal,1/(self.momentXX+self.momentYY)*self.Lr*self.Ki*(u[0]-u[1]+u[2]-u[3])])

        return retVal*self.timeDelta

    def getF(self, x,u):

        # So state-transition Jacobian is identity matrix
        #qx
        retVal=np.array([0,0,0,1,0,0,0,0,0,0,0,0])
        #qy
        retVal=np.vstack([retVal,[0,0,0,0,1,0,0,0,0,0,0,0]])
        #qz
        retVal=np.vstack([retVal,[0,0,0,0,0,1,0,0,0,0,0,0]])
        #vx
        retVal=np.vstack([retVal,[0,0,0,-self.b/self.mass,0,0,0,self.Kf/self.mass*math.cos(x[7])*sum(u),0,0,0,0]])
        #vy
        retVal=np.vstack([retVal,[0,0,0,0,-self.b/self.mass,0,self.Kf/self.mass*math.cos(x[7])*math.cos(x[6])*sum(u),-self.Kf/self.mass*math.sin(x[7])*math.sin(x[6])*sum(u),0,0,0,0]])
        #vz
        retVal=np.vstack([retVal,[0,0,0,0,0,-self.b/self.mass,-self.Kf/self.mass*math.sin(x[7])*math.sin(x[6])*sum(u),self.Kf/self.mass*math.cos(x[7])*math.cos(x[6])*sum(u),0,0,0,0]])
        #qtheta
        retVal=np.vstack([retVal,[0,0,0,0,0,0,0,0,0,1,0,0]])
        #qphi
        retVal=np.vstack([retVal,[0,0,0,0,0,0,0,0,0,0,1,0]])
        #qpsi
        retVal=np.vstack([retVal,[0,0,0,0,0,0,0,0,0,0,0,1]])
        #vtheta
        retVal=np.vstack([retVal,[0,0,0,0,0,0,0,0,0,0,-x[11],-x[10]]])
        #vphi
        retVal=np.vstack([retVal,[0,0,0,0,0,0,0,0,0,x[11],0,x[9]]])
        #vpsi
        retVal=np.vstack([retVal,[0,0,0,0,0,0,0,0,0,0,0,1]])

        return retVal

    def h(self, x):

        # Observation function is identity
        return np.ones([13,1])

    def getH(self, x):

        # So observation Jacobian is identity matrix
        #return np.eye(13)
        return np.ones([13,12])

if __name__ == '__main__':
    droneEKF = DroneEKF()
    testZ = (0,0,0,0,0,0,0,0,0,0,0,0,0)
    testU=(0,0,0,0)
    droneEKF.step(testZ,testU)
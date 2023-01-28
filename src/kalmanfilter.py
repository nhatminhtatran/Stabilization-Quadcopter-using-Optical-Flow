import numpy as np
class SimpleKalmanFilter:
    def __init__(self, mea_e, est_e, q):
        self.mea_e = mea_e # Measurement Uncertainty - How much do we expect to our measurement vary
        self.est_e = est_e #Estimation Uncertainty - Can be initilized with the same value as e_mea since the kalman filter will adjust its value.
        self.q = q # Process Variance - usually a small number between 0.001 and 1 - how fast your measurement moves. Recommended 0.01. Should be tunned to your needs.
        self.last_est = 0
  
    def update(self, mea):# mea is measure ment
        kalman_gain = self.est_e/(self.est_e + self.mea_e) # calculation kalman gain
        current_est = self.last_est + kalman_gain*(mea - self.last_est) # estimating the current state

        self.est_e = (1.0 - kalman_gain) * self.est_e + np.fabs(self.last_est - current_est) * self.q #update the current uncertainty
        self.last_est = current_est
        return current_est
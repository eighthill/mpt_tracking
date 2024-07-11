# -*- coding: utf-8 -*-
"""
Created on Wed Jul 10 03:38:31 2024

@author: marko
"""
import numpy as np
from math import sin, cos

class ConstantVelocity2:
    def __init__(self, guess_H = 1, guess_P = 1, Q_noise = 0.0005):
        #self.state_dim = state_dim
        self.dt = 1
        self.state_estimate = np.zeros(4)
        self.guess_P = guess_P
        self.guess_H = guess_H
        self.Q_noise = Q_noise
        self.P = np.eye(4) * guess_P        
        #self.state_transition = np.eye(4)
        self.H = np.eye(2,4) * guess_H #measurement_matrix
        self.Q = np.eye(4) * self.Q_noise
        self.I = np.eye(4) #einheitsmatrix
        
        
        """in den folien:
             state = F@state + Ga
             mit:
                 Ga = np.array([[0.5*dt**2, 0],
                                [0, 0.5*dt**2], 
                                [dt, 0],
                                [0, dt]])
                 Q = np.ndarray.var(Ga)"""
            

    def reset(self, measurement):
        self.state_estimate[:2] = np.mean(measurement[:10].reshape(-1, 2), axis=0) #positions
        self.state_estimate[2:] = 0  # velocity: unknown
        self.P = np.eye(4) # uncartainty covariance
        return self.state_estimate[:2]
    
    def update(self, dt, measurement):
        #Prediciton
        self.dt = dt
        
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]]) #Prozessmodell
        
        G = np.array([[0.5*dt**2, 0],
                       [0, 0.5*dt**2], 
                       [dt, 0],
                       [0, dt]])
        Q = np.ndarray.var(G) * self.Q_noise
        
        self.state_estimate = F @ self.state_estimate 
        
        #Q = self.Q * self.Q_noise
        
        self.P = F @ self.P @ F.T + Q #hat shape (4,4)
        
        #getting values
        measured_values = measurement[:10].reshape(-1, 2)
        R = measurement[10:].reshape(-1, 2) #measurement_noise_covariance/measurement_noise
        #R = np.diag(np.concatenate([R_values[:, 0]**2, R_values[:, 1]**2]))
        
        #avg because we have multiple measurements
        avg_value = np.mean(measured_values, axis = 0)
        avg_R = np.ones((2,2))
        
        n_measured_values = len(measured_values)
        
        for i in range(n_measured_values):
            avg_R *= np.diag(R[i]) **2
        avg_R = avg_R/ len(measured_values)

        #-------calculating----------
        
        #Innovation
        #H = np.eye(2,4)
        residual_covariance = self.H @ self.P @ self.H.T + avg_R # means S
        #KalmanGain
        kalman_gain = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(residual_covariance))
        
        measurement_residual = avg_value - self.H @ self.state_estimate # innovation
        
        #Update
        self.state_estimate = self.state_estimate + kalman_gain @ measurement_residual
        self.P = np.dot((self.I - np.dot(kalman_gain, self.H)), self.P)
        
        return self.state_estimate[:2]#.reshape(1, 2)
    
class ConstantVelocity():
    def __init__(self, process_noise=1e-6, measurement_noise=0.69):
        """Initialize Constant Velocity Kalman Filter with default values."""
        self.dt = 1  # Default time step
        self.state = np.zeros(4)  # Initial state vector [x, y, vx, vy]
        self.uncertainty = np.eye(4)  # Initial uncertainty covariance matrix
        self.process_noise = process_noise  # Process noise
        self.measurement_noise = measurement_noise  # Measurement noise

    def reset(self, measurement):
        """Reset the filter with an initial measurement."""
        self.state[:2] = np.array(measurement[:2])  # Initialize position part of the state
        self.state[2:] = 0  # Initial velocity is unknown, set to zero
        self.uncertainty = np.eye(4) / 2  # Reset uncertainty
        return self.state[:2]  # Return only the position part

    def predict(self, dt):
        """Predict the next state and uncertainty."""
        self.dt = dt
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
        Q = np.eye(4) * self.process_noise  # Process noise covariance matrix

        self.state = F @ self.state  # State prediction
        self.uncertainty = F @ self.uncertainty @ F.T + Q  # Uncertainty prediction

    def calculate_kalman_gain(self, measurement_uncertainty):
        """Calculate the Kalman Gain."""
        H = np.eye(2, 4)  # Measurement matrix (mapping state to measurement space)
        return (
            self.uncertainty
            @ H.T
            @ np.linalg.inv(H @ self.uncertainty @ H.T + measurement_uncertainty)
        )

    def update_state(self, measurement, kalman_gain):
        """Update the state estimate."""
        H = np.eye(2, 4)  # Measurement matrix
        self.state = self.state + kalman_gain @ (measurement - H @ self.state)

    def update_uncertainty(self, kalman_gain):
        """Update the uncertainty covariance."""
        H = np.eye(2, 4)  # Measurement matrix
        self.uncertainty = (np.eye(4) - kalman_gain @ H) @ self.uncertainty

    def update(self, dt, measurement):
        """Perform a full update cycle: predict and update."""
        self.predict(dt)

        measurement = np.array(measurement[:2])
        measurement_uncertainty = np.eye(2) * self.measurement_noise**2

        kalman_gain = self.calculate_kalman_gain(measurement_uncertainty)

        self.update_state(measurement, kalman_gain)

        self.update_uncertainty(kalman_gain)

        return self.state[:2]
    
    """def __init__(self, guess_H = 1, guess_P = 1, Q_noise = 0.0005):
        #self.state_dim = state_dim
        self.dt = 1
        self.state_estimate = np.zeros(4)#np.zeros(2)
        self.guess_P = guess_P
        self.guess_H = guess_H
        self.Q_noise = Q_noise
        #self.P = np.eye(2) * guess_P        
        #self.state_transition = np.eye(4)
        self.H = np.eye(2,4)#np.array([1,0])#np.eye(2)# * guess_H #measurement_matrix
        self.Q = np.eye(4) * self.Q_noise + self.dt
        self.P = np.eye(4)
        #self.I = np.array([[1, 0],
                          #[0,1]]) #einheitsmatrix
        

        
        
        
            

    def reset(self, measurement):
        #self.state_estimate = np.array([measurement[:2]])
        self.state_estimate[:2] = np.mean(measurement[:10].reshape(-1, 2), axis=0) #positions
        self.state_estimate[2:] = 0  # velocity: unknown
        #self.P = np.eye(4)/2 # uncartainty covariance
        #self.H = np.eye(2)
        return self.state_estimate[:2]
    
    def update(self, dt, measurement):
        
        #Prediciton
        #print(f" \n H :{self.H}\nP.{self.P}\n")
        self.dt = dt
        
        F = np.array([[1, dt],
                      [0, 1]]) #Prozessmodell
        ##F = np.array([[1, 0, dt, 0],
         #             [0, 1, 0, dt],
          #            [0, 0, 1, 0],
             #         [0, 0, 0, 1]])
        
        #G = np.array([[0.5*dt**2, 0],
         #              [0, 0.5*dt**2], 
          #             [dt, 0],
           #            [0, dt]])
        Q = self.Q#np.ndarray.var(G) * self.Q_noise
        print(F.shape, self.state_estimate.shape)
        print(self.state_estimate)
        self.state_estimate = F @ self.state_estimate
        
        #Q = self.Q * self.Q_noise
        self.P = F @ self.P @ F.T + Q 
        
        #getting values
        measured_values = np.mean(measurement[:10].reshape(-1, 2), axis=0)
        R = np.eye(2) * 0.5**2#setting constant velocity#.reshape(-1, 2) #measurement_noise_covariance/measurement_noise
        #print(measured_values)
        
        #Innovation
        residual_covariance = self.H @ self.P @ self.H.T + R # means S
        
        #KalmanGain
        kalman_gain = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(residual_covariance))
        #print(f"KG: {kalman_gain}")
        
        measurement_residual = measured_values - self.H @ self.state_estimate # innovation
        
        #Update
        self.state_estimate = self.state_estimate + kalman_gain @ measurement_residual
        
        I = np.eye(4)
        
        self.P = np.dot((I - np.dot(kalman_gain, self.H)), self.P)
        
        
        return self.state_estimate[:2]#.reshape(1, 2)"""
    
    
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 25 11:50:45 2024

@author: marko
"""

import numpy as np

class staticFilter():
    def __init__(self, shape):  
        self.shape = shape
        self.measurement_variance = np.eye(1) * 0.04
        self.state_variance = np.eye(1) * 100 #random value
        self.state_estimate = np.zeros(1)
        

    def reset(self, measurement):    
        return measurement[:2]
    
    def update(self, dt, measurement):  
        num = len(measurement)
        for i in range(num):
            t = self.measurement_variance/(self.measurement_variance + self.state_variance)
            self.state_estimate = t*self.state_estimate + (1-t) * measurement
            self.state_variance = t**2 * self.state_variance + (1-t)**2 * self.measurement_variance
            
            
        return np.array([self.state_estimate[0][0],self.state_estimate[1][1]]) #otherwise it return an 2x2 Array
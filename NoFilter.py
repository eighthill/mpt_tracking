# -*- coding: utf-8 -*-
"""
Created on Fri Jun 21 19:48:54 2024

@author: marko
"""

class NoFilter():
  def __init__(self):
    pass

  def reset(self, measurement):    
    return measurement[:2]
  
  def update(self, dt, measurement):  
    return measurement[:2]
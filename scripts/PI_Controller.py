#!/usr/bin/env python
import os
import math
import numpy as np
import matplotlib.pyplot as plt


def PI_Controller(x_desired, x_real, K_P, K_I, I0):
    
    e = x_desired - x_real                #Error between the real and desired value 
    P = K_P * e                           #Proportional controller 
    #I = I0 + K_I * e * step              #Integral controller
    I = I0 + e
    Tau = P + K_I * I                    #Output of the PID controller 
    I0 = I                               #Update the initial value of integral controller 
    
    return Tau
    
def PI_Controller_With_Comp(x_desired, x_real, K_P, K_I, step, I0,flotability):
    
    e =  x_desired - x_real              			 #Error between the real and desired value 
    P = K_P * e                         			 #Proportional controller 
    I = I0 +  e * step              			     #Integral controller
    PI_Controller = P + K_I * I + flotability        #Output of the PID controller 
    I0 = I                                          #Update the initial value of integral controller 
    
    return PI_Controller 

def PID_Controller_With_Comp(x_desired, x_real, K_P, K_I, K_D, e_0, I0, step,flotability, r = None):
    
    e =  x_desired - x_real               #Error between the real and desired value
    P = K_P * e                           #Proportional controller 
    if r == None : 
        D = K_D * ((e - e_0)/ step)
    else : 
        D = K_D * r

    I = I0 +  e * step                                                 #Integral controller
    PID_Controller = P + K_I * I + D + flotability                     #Output of the PID controller 
    e_0 = e
    I0 = I                                                             #Update the initial value of integral controller 

    return PID_Controller 



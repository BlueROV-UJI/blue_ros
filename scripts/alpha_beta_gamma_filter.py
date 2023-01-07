#!/usr/bin/env python
import os
import math
import numpy as np
import matplotlib.pyplot as plt

def alpha_beta_gamma_filter(x_e0, v_e0, a_e0, x_m, alpha, beta, step):
    gamma = beta**2 / (2 * alpha) 
    x_e = x_e0 + v_e0 * step
    v_e = v_e0 + a_e0 * step 
    #a_e = (v_e - v_e0) / step
    r = x_m - x_e 
    
    #estimate pos
    x_e += alpha * r
    
    #estimate speed
    v_e += (beta / step) * r
    
    #acceleration 
   # a_e += a_e0 + (2 * gamma/ step) * r 
    return v_e, x_e

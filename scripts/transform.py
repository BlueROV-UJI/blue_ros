#!/usr/bin/env python3
import math
from os import kill
import string
import numpy as np





def rotXYZ(rx,ry,rz, degrees =True):
    if(degrees):
        rx *= math.pi/180
        ry *= math.pi/180
        rz *= math.pi/180
    rotx = np.array([[1,0,0],
                     [0,math.cos(rx),-math.sin(rx)],
                     [0,math.sin(rx),math.cos(rx)]])
    
    roty = np.array([[math.cos(ry),0,math.sin(ry)],
                     [0,1,0],
                     [-math.sin(ry),0, math.cos(ry)]])
    
    rotz = np.array([[math.cos(rz),-math.sin(rz),0],
                     [math.sin(rz),math.cos(rz),0],
                     [0,0,1]])
    rotxyz = rotx.dot(roty.dot(rotz))
   # print('r',rotxyz)
    rotxyz = np.round (rotxyz,15)
    #print('round',rotxyz)
    return rotxyz

def isRot (M, dec):
    tag = False
    I= np.identity(M.shape[0])
    
    print('cond1 M.MT=I',np.round (np.matmul(M, M.T)),dec)
    print('cond2 np.linalg.det(M)',np.round(np.linalg.det(M),dec))
    if (np.all(np.round( (np.matmul(M, M.T)),10) == I) and (np.round(np.linalg.det(M),dec)==1)): 
        tag = True

    return tag
    


# define the transformations from one frame c to frameo
def homogenousMatrix(tx,ty,tz,rx,ry,rz):
    cRo = rotXYZ(rx,ry,rz, degrees =True)
    cTo = np.array([[tx,ty,tz]])
    M = np.concatenate(
        (np.concatenate((cRo,cTo.T), axis=1),
         np.array([[0,0,0,1]])),axis=0 )
    return M

# skew a 3x1 vector in a matrix
def skew_vec (vec):
    mat = np.array([[0,-vec[2], vec[1]],[vec[2],0,-vec[0]],[-vec[1], vec[0], 0]])
    return mat


# expressed a velocity in frame a knowing velocity in b an
# change of frame aMb
def velocityTwistMatrix(tx,ty,tz,rx=0,ry=0,rz=0, aRb=None):
    aRb = rotXYZ(rx,ry,rz, degrees=True) if aRb is None else aRb
    aTb_x = skew_vec(np.array([tx,ty,tz]))
    aTb_xaRb = aTb_x.dot(aRb)
    aVb1 = np.concatenate((aRb,aTb_xaRb),axis=1)
    aVb2 = np.concatenate((np.zeros((3,3)),aRb),axis=1) 
    aVb = np.concatenate ((aVb1,aVb2))
    return aVb
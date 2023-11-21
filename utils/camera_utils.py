'''
Author: BigCiLeng && bigcileng@outlook.com
Date: 2023-11-21 20:16:09
LastEditors: BigCiLeng && bigcileng@outlook.com
LastEditTime: 2023-11-21 22:27:00
FilePath: /pointcloud_projection/utils/camera_utils.py
Description: 

Copyright (c) 2023 by bigcileng@outlook.com, All Rights Reserved. 
'''
import numpy as np
def rotation_metric(axis):
    rot_phi_x = lambda phi : np.array([
        [1,0,0,0],
        [0,np.cos(phi),-np.sin(phi),0],
        [0,np.sin(phi), np.cos(phi),0],
        [0,0,0,1]
    ])
    rot_phi_y = lambda phi : np.array([
        [np.cos(phi),0,np.sin(phi),0],
        [0,1,0,0],
        [-np.sin(phi),0, np.cos(phi),0],
        [0,0,0,1]
    ])    
    rot_phi_z = lambda phi : np.array([
        [np.cos(phi),-np.sin(phi),0,0],
        [np.sin(phi), np.cos(phi),0,0],
        [0,0,1,0],
        [0,0,0,1]
    ])
    if axis == 0:
        return rot_phi_x
    elif axis == 1:
        return rot_phi_y
    elif axis == 2:
        return rot_phi_z
    assert "axis error"

def camera_rotation(C2W, phi, axis):

    W2C = np.linalg.inv(C2W)
    rotation = rotation_metric(axis)
    W2C = rotation(phi) @ W2C

    C2W = np.linalg.inv(W2C)
    return C2W
def camera_revolution(C2W, phi, axis):
    rotation = rotation_metric(axis)
    C2W = rotation(phi) @ C2W

    return C2W

def camera_translation(C2W, t, axis):
    C2W[axis,3] += t
    return C2W
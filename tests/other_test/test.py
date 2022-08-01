#!/usr/bin/env python
# coding: utf-8
import transforms3d as tfs
import numpy as np
import math

def get_matrix_eular_radu(x,y,z,rx,ry,rz):
    rmat = tfs.euler.euler2mat(math.radians(rx),math.radians(ry),math.radians(rz))
    rmat = tfs.affines.compose(np.squeeze(np.asarray((x,y,z))), rmat, [1, 1, 1])
    return rmat



"""
   /* base@grapper  */
    Matrix4d Tbg = TransForms::ComposeEuler(-0.544, -0.203,-0.037, 180, 0.00000, 140);
    /* grapper@camera */
    Matrix4d Tgc  = TransForms::ComposeEuler(0.056,-0.056,-0.106,0,-0,45);
    /*  camera@maker*/
    Matrix4d Tcw  = TransForms::ComposeEuler(0.056,-0.056,-0.106,0,-0,45);
    /*  base@bottle  */
    Matrix4d Tbw  = TransForms::ComposeEuler(-0.663,-0.193,-0.231,-180,0,140);
# Tcg = np.dot(np.linalg.pinv(MA),MB).reshape(3,)
# print(tfs.affines.compose(Tcg,np.squeeze(Rcg),[1,1,1]))
"""
Tbg = get_matrix_eular_radu(-0.544, -0.203,-0.037, 180, 0.00000, 140)
Tgc = get_matrix_eular_radu(0.056,-0.056,-0.106,0,0,45)
Tcw = get_matrix_eular_radu(0.020,-0.040,0.300,0,0,-45)
Tbc = np.dot(Tbg,Tgc)
Tbw = np.dot(Tbc,Tcw)
print(Tbw)
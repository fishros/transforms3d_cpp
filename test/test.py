#!/usr/bin/env python
# coding: utf-8
import transforms3d as tfs
import numpy as np
import math

def get_matrix_eular_radu(x,y,z,rx,ry,rz):
    rmat = tfs.euler.euler2mat(math.radians(rx),math.radians(ry),math.radians(rz))
    rmat = tfs.affines.compose(np.squeeze(np.asarray((x,y,z))), rmat, [1, 1, 1])
    return rmat




# Tcg = np.dot(np.linalg.pinv(MA),MB).reshape(3,)
# print(tfs.affines.compose(Tcg,np.squeeze(Rcg),[1,1,1]))

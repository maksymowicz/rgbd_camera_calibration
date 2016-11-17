import numpy as np
import numpy.linalg
import scipy as sp
import scipy.linalg
import warnings

def rotation_matrix_to_quaternion(R):

    warnings.filterwarnings("error")
    
    # matrix logarithm to get skew symmetric
    S = sp.linalg.logm(R).real

    # get values
    x = S[2, 1]
    y = S[0, 2]
    z = S[1, 0]

    # euler axis
    e = np.array([x, y, z])

    # compute rotation
    theta = np.linalg.norm(e)

    # normalize 
    try:
        e = e / theta
    except RuntimeWarning:
        e = np.zeros(3)

    # build quaternion
    q = np.zeros(4)
    q[0] = np.cos(theta / 2.)
    q[1:] = np.sin(theta / 2.) * e

    return q

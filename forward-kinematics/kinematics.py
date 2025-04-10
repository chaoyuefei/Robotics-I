# kinematics.py
import numpy as np
from scipy.spatial.transform import Rotation as R

def dh_transform(theta, d, a, alpha):
    """
    Compute the homogeneous transformation matrix using DH parameters.

    Args:
        theta: Joint angle (in radians)
        d: Link offset
        a: Link length
        alpha: Link twist

    Returns:
        4x4 Homogeneous transformation matrix
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,       sa,      ca,      d],
        [0,        0,       0,      1]
    ])



# kinematics.py
def forward_kinematics(dh_params, return_all=False):
    """
    Compute the forward kinematics based on a DH table.

    Args:
        dh_params: list of dictionaries, each containing keys ['theta', 'd', 'a', 'alpha']
        return_all: if True, returns intermediate transforms

    Returns:
        T (final transformation matrix) or list of intermediate transforms if return_all=True
    """
    T = np.eye(4)  # Identity matrix
    Ts = [T]
    for param in dh_params:
        T = T @ dh_transform(param['theta'], param['d'], param['a'], param['alpha'])
        Ts.append(T)
    
    if return_all:
        return Ts
    else:
        return T


# kinematics.py
def end_effector_position(T):
    """
    Extract the position of the end-effector from the transformation matrix.

    Args:
        T: Homogeneous transformation matrix

    Returns:
        position: 3D position of the end-effector (x, y, z)
    """
    return T[:3, 3]

def end_effector_orientation(T, degrees=True):
    """
    Extract the orientation (Euler angles) of the end-effector from the transformation matrix.

    Args:
        T: Homogeneous transformation matrix
        degrees: whether to return angles in degrees (default=True)

    Returns:
        Euler angles (yaw, pitch, roll) in the specified units
    """
    r = R.from_matrix(T[:3, :3])  # Convert rotation matrix to rotation object
    return r.as_euler('zyx', degrees=degrees)

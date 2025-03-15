"""!
Implements Forward and Inverse kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""

import numpy as np
# expm is a matrix exponential function
from scipy.linalg import expm


def clamp(angle):
    """!
    @brief      Clamp angles between (-pi, pi]

    @param      angle  The angle

    @return     Clamped angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle

def dh_matrix_list_gen(theta, d, a, alpha):
    if not (len(theta) == len(d) == len(a) == len(alpha)):
        raise ValueError("non-equal dh params.")
    
    n = len(theta)
    dh_matrices = []

    for i in range(n):
        matrix = np.array([
            [np.cos(theta[i]), -np.sin(theta[i]) * np.cos(alpha[i]),  np.sin(theta[i]) * np.sin(alpha[i]), a[i] * np.cos(theta[i])],
            [np.sin(theta[i]),  np.cos(theta[i]) * np.cos(alpha[i]), -np.cos(theta[i]) * np.sin(alpha[i]), a[i] * np.sin(theta[i])],
            [0,                 np.sin(alpha[i]),                   np.cos(alpha[i]),                   d[i]],
            [0,                 0,                                  0,                                  1]
        ])
        dh_matrices.append(matrix)

    return np.array(dh_matrices)


def comp_forward_kin(dh_matrices):
    if not isinstance(dh_matrices, np.ndarray) or len(dh_matrices.shape) != 3 or dh_matrices.shape[1:] != (4, 4):
        raise ValueError("Input must be a np array with shape (n, 4, 4).")
    
    cumulative_matrix = np.eye(4)
    
    for i, matrix in enumerate(dh_matrices):
        cumulative_matrix = np.dot(cumulative_matrix, matrix)
        
        position = cumulative_matrix[:3, 3]
        #print(f"Step {i+1}: x = {position[0]:.4f}, y = {position[1]:.4f}, z = {position[2]:.4f}")
    return cumulative_matrix


def FK_dh(dh_params, joint_angles, link):
    """!
    @brief      Get the 4x4 transformation matrix from link to world

                TODO: implement this function

                Calculate forward kinematics for rexarm using DH convention

                return a transformation matrix representing the pose of the desired link

                note: phi is the euler angle about the y-axis in the base frame

    @param      dh_params     The dh parameters as a 2D list each row represents a link and has the format [a, alpha, d,
                              theta]
    @param      joint_angles  The joint angles of the links
    @param      link          The link to transform from

    @return     a transformation matrix representing the pose of the desired link
    """

    theta = dh_params[:,3]
    a = dh_params[:,0]
    alpha = dh_params[:,1]
    d = dh_params[:,2]

    theta = theta + joint_angles

    dh_matrix_list = dh_matrix_list_gen(theta, d, a, alpha)

    return comp_forward_kin(dh_matrix_list)


def get_transform_from_dh(a, alpha, d, theta):
    """!
    @brief      Gets the transformation matrix T from dh parameters.

    TODO: Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transformation matrix.
    """
    pass


def get_euler_angles_from_T(T):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO: Implement this function return the 3 Euler angles from a 4x4 transformation matrix T
                If you like, add an argument to specify the Euler angles used (xyx, zyz, etc.)

    @param      T     transformation matrix

    @return     The euler angles from T.
    """
    psi = np.arctan2(T[1,2],-T[0,2])
    theta = np.arccos(T[2,2])
    phi = np.arctan2(T[2,1], T[2,0])

    return [phi, theta, psi]


def get_pose_from_T(T):
    """!
    @brief      Gets the pose from T.

                TODO: implement this function return the 6DOF pose vector from a 4x4 transformation matrix T

    @param      T     transformation matrix

    @return     The pose vector from T.
    """

    position = T[:3, 3]
    phi = np.arctan2(T[1,2],-T[0,2])
    theta = np.arccos(T[2,2])
    psi = np.arctan2(T[2,1], T[2,0])

    return [position[0], position[1], position[2], phi, theta, psi]


def FK_pox(joint_angles, m_mat, s_lst):
    """!
    @brief      Get a  representing the pose of the desired link

                TODO: implement this function, Calculate forward kinematics for rexarm using product of exponential
                formulation return a 4x4 homogeneous matrix representing the pose of the desired link

    @param      joint_angles  The joint angles
                m_mat         The M matrix
                s_lst         List of screw vectors

    @return     a 4x4 homogeneous matrix representing the pose of the desired link
    """
    pass


def to_s_matrix(w, v):
    """!
    @brief      Convert to s matrix.

    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)

    @param      w     { parameter_description }
    @param      v     { parameter_description }

    @return     { description_of_the_return_value }
    """
    pass


def IK_geometric(dh_params, pose, block_orient):
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose vector as np.array to joint angles

    @param      dh_params  The dh parameters
    @param      pose       The desired pose vector as np.array 

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """


    x,y,z,psi = pose
    if z > 90:
        z += 50
    elif z > 180:
        z += 60
    psi = psi - np.pi/2

    l1 = dh_params[0][2]
    l2 = dh_params[1][0]
    l3 = dh_params[2][0]
    l4 = dh_params[4][2]
 
    z = z -l1

    theta1 = -1*np.arctan2(x,y)

    D = np.sqrt(x**2+y**2)
    D_hat = D - l4*np.cos(psi)  
    z_hat = z + l4*np.sin(psi) 
    D_hat -= 10

    r = np.sqrt(D_hat**2+z_hat**2)

    # Check if targtet pose is reachable (see if triangle can be formed)
    if r > (l2 + l3):
        print("Pose is unreachable!")
        return False, [0, 0, 0, 0, 0]

    theta2 = np.pi/2 - np.arctan2(50, 200) - np.arctan2(z_hat,D_hat) - np.arccos((r**2 + l2**2 - l3**2)/(2*r*l2))

    elbow = np.arccos((l2**2 + l3**2 - r**2)/(2*l2*l3))

    theta3 = np.pi/2 + np.arctan2(50, 200) - elbow

    theta4 = psi - (theta2 + theta3)

    theta5 = 0

    if psi == 0:
        theta5 = 0
    elif theta1 > 0:
        #theta5 = block_orient % (np.pi/4) + np.pi/2 - theta1
        theta5 = block_orient - theta1
    elif theta1 < 0:
        #theta5 = (block_orient % (np.pi/4)) - np.pi/2 + abs(theta1)
        if abs(theta1) < np.pi/2:
            theta5 = block_orient + theta1
        else:
            theta5 = theta1 + block_orient
        



    # Check if joint angles are within valid ranges
    if theta1 >= np.pi or theta1 <= -np.pi:
        return False, [0, 0, 0, 0, 0]

    if theta2 >= np.deg2rad(113) or theta2 <= -np.deg2rad(108):
        return False, [0, 0, 0, 0, 0]

    if theta3 >= np.deg2rad(93) or theta3 <= -np.deg2rad(108):
        return False, [0, 0, 0, 0, 0]

    if theta4 >= np.deg2rad(123) or theta4 <= -np.deg2rad(100):
        return False, [0, 0, 0, 0, 0]

    return True, [theta1, theta2, theta3, theta4, theta5]

import numpy as np
import matplotlib.pyplot as plt
import math
from sympy import pi, Matrix, symbols, cos, sin

def scara_FK(theta_1, d_1, theta_3, theta_4):


    theta_1_rad = math.radians(theta_1)
    theta_3_rad = math.radians(theta_3)
    theta_4_rad = math.radians(theta_4)
    d_1 = d_1 + 123 #BASE OFFSET

    
    a_1_val, alpha_1_val, d_1_val, theta_1_val = 0, 0, d_1, -theta_1_rad+pi/2
    a_2_val, alpha_2_val, d_2_val, theta_2_val = 329.498, 0, 0, 0
    a_3_val, alpha_3_val, d_3_val, theta_3_val = 285.0, pi, 0, -theta_3_rad
    a_4_val, alpha_4_val, d_4_val, theta_4_val = 25.61, 0, 102.0, theta_4_rad #TCP offsets

    # Homogeneous Transformation Matrices
    H_dh_1 = Matrix([
        [cos(theta_1_val), -sin(theta_1_val)*cos(alpha_1_val), sin(theta_1_val)*sin(alpha_1_val), a_1_val*cos(theta_1_val)],
        [sin(theta_1_val), cos(theta_1_val)*cos(alpha_1_val), -cos(theta_1_val)*sin(alpha_1_val), a_1_val*sin(theta_1_val)],
        [0, sin(alpha_1_val), cos(alpha_1_val), d_1_val],
        [0, 0, 0, 1]
    ])

    H_dh_2 = Matrix([
        [cos(theta_2_val), -sin(theta_2_val)*cos(alpha_2_val), sin(theta_2_val)*sin(alpha_2_val), a_2_val*cos(theta_2_val)],
        [sin(theta_2_val), cos(theta_2_val)*cos(alpha_2_val), -cos(theta_2_val)*sin(alpha_2_val), a_2_val*sin(theta_2_val)],
        [0, sin(alpha_2_val), cos(alpha_2_val), d_2_val],
        [0, 0, 0, 1]
    ])

    H_dh_3 = Matrix([
        [cos(theta_3_val), -sin(theta_3_val)*cos(alpha_3_val), sin(theta_3_val)*sin(alpha_3_val), a_3_val*cos(theta_3_val)],
        [sin(theta_3_val), cos(theta_3_val)*cos(alpha_3_val), -cos(theta_3_val)*sin(alpha_3_val), a_3_val*sin(theta_3_val)],
        [0, sin(alpha_3_val), cos(alpha_3_val), d_3_val],
        [0, 0, 0, 1]
    ])

    H_dh_4 = Matrix([
        [cos(theta_4_val), -sin(theta_4_val)*cos(alpha_4_val), sin(theta_4_val)*sin(alpha_4_val), a_4_val*cos(theta_4_val)],
        [sin(theta_4_val), cos(theta_4_val)*cos(alpha_4_val), -cos(theta_4_val)*sin(alpha_4_val), a_4_val*sin(theta_4_val)],
        [0, sin(alpha_4_val), cos(alpha_4_val), d_4_val],
        [0, 0, 0, 1]
    ])


    H_dh_01 = H_dh_1
    H_dh_02 = H_dh_1 * H_dh_2
    H_dh_03 = H_dh_1 * H_dh_2 * H_dh_3
    H_dh_04 = H_dh_1 * H_dh_2 * H_dh_3 * H_dh_4

    x = H_dh_04[0, 3]
    y = H_dh_04[1, 3]
    z = H_dh_04[2, 3]

    g = theta_1 + theta_3 + theta_4

    return x, y, z, g










import numpy as np
import matplotlib.pyplot as plt
import math
from sympy import pi, Matrix, symbols, cos, sin


def calculate_forward_kinematics(joints):

    theta_1_rad = math.radians(joints[0])
    d_1 = joints[1] + 123 #BASE OFFSET
    theta_3_rad = math.radians(joints[2])
    theta_4_rad = math.radians(joints[3])

    link_1 = 329.498
    link_2 = 285.0

    TCP_vertical_offset = 102.0
    TCP_horizontal_offset = 110.0

    # Define values for the symbolic variables
    a_1_val, alpha_1_val, d_1_val, theta_1_val = 0, 0, d_1, -theta_1_rad+pi/2
    a_2_val, alpha_2_val, d_2_val, theta_2_val = link_1, 0, 0, 0
    a_3_val, alpha_3_val, d_3_val, theta_3_val = link_2, pi, 0, -theta_3_rad
    a_4_val, alpha_4_val, d_4_val, theta_4_val = TCP_horizontal_offset, 0, TCP_vertical_offset, theta_4_rad

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

    link1_x = [0, H_dh_01[0,3]]
    link1_y = [0, H_dh_01[1,3]]
    link1_z = [0, H_dh_01[2,3]]

    link2_x = [H_dh_01[0,3], H_dh_02[0,3]]
    link2_y = [H_dh_01[1,3], H_dh_02[1,3]]
    link2_z = [H_dh_01[2,3], H_dh_02[2,3]]

    link3_x = [H_dh_02[0,3], H_dh_03[0,3]]
    link3_y = [H_dh_02[1,3], H_dh_03[1,3]]
    link3_z = [H_dh_02[2,3], H_dh_03[2,3]]

    link4_x_v1 = [H_dh_03[0,3], H_dh_03[0,3]]
    link4_y_v1 = [H_dh_03[1,3], H_dh_03[1,3]]
    link4_z_v1 = [H_dh_03[2,3], H_dh_03[2,3]-TCP_vertical_offset ]

    link4_x_v2 = [H_dh_03[0,3], H_dh_04[0,3]]
    link4_y_v2 = [H_dh_03[1,3], H_dh_04[1,3]]
    link4_z_v2 = [H_dh_03[2,3]-TCP_vertical_offset , H_dh_04[2,3]]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    return link1_x, link1_y, link1_z, link2_x, link2_y, link2_z, link3_x, link3_y, link3_z, link4_x_v1, link4_y_v1, link4_z_v1, link4_x_v2, link4_y_v2, link4_z_v2









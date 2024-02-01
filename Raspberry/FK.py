import numpy as np
import matplotlib.pyplot as plt
import math
from sympy import pi, Matrix, symbols, cos, sin
#test

def scara_FK(theta_1, d_1, theta_3, theta_4):


    theta_1_rad = math.radians(theta_1)
    theta_3_rad = math.radians(theta_3)
    theta_4_rad = math.radians(theta_4)
    d_1 = d_1 + 123 #BASE OFFSET


    # Define values for the symbolic variables
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

    # print(H_dh_04)

    x = H_dh_04[0, 3]
    y = H_dh_04[1, 3]
    z = H_dh_04[2, 3]

    g = theta_1 + theta_3 + theta_4

    return x, y, z, g

    print("x:", x)
    print("y:", y)
    print("z:", z)

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
    link4_z_v1 = [H_dh_03[2,3], H_dh_03[2,3]-102.0]

    link4_x_v2 = [H_dh_03[0,3], H_dh_04[0,3]]
    link4_y_v2 = [H_dh_03[1,3], H_dh_04[1,3]]
    link4_z_v2 = [H_dh_03[2,3]-102.0, H_dh_04[2,3]]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the line segments
    ax.plot(link1_x, link1_y, link1_z, 'b-', linewidth=20)
    ax.plot(link2_x, link2_y, link2_z, 'b-', linewidth=10)
    ax.plot(link3_x, link3_y, link3_z, 'b-', linewidth=5)
    ax.plot(link4_x_v1, link4_y_v1, link4_z_v1, 'b-', linewidth=2)
    ax.plot(link4_x_v2, link4_y_v2, link4_z_v2, 'b-', linewidth=2)
    ax.plot(x, y, z, 'b-', linewidth=5)

    # Set axis limits
    ax.set_xlim([-700, 700])
    ax.set_ylim([-700, 700])
    ax.set_zlim([0, 500])

    # Set axis labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Show the plot
    plt.show()






x,y,z,theta_4 = scara_FK(0, 0, 0, 0)

print("X:",x)
print("Y:",y)
print("Z:",z)
print("THETA4:",theta_4)

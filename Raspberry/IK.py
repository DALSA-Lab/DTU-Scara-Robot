import math

def calculate_scara_angles(x, y, z, gripper_or, pos):
    l_1 = 329.5
    l_2 = 285.0
    l_3 = 25.65 # TCP vertical offset
    l_4 = 102.0 # TCP horizontal offset
    # l_3 = 0.0 # TCP vertical offset
    # l_4 = 0.0 # TCP horizontal offset
    d_b = 123.0

    print("X coord:", x)
    print("Y coord:", y)
    print("Z coord:", z)
    print("A coord:", gripper_or)
    print("POS: ", pos)

    gripper_or_radians = math.radians(gripper_or)
    
    #TCP change
    d_x = l_3*math.sin(gripper_or_radians)
    x = x - d_x

    d_y = l_3*math.cos(gripper_or_radians)
    y = y - d_y

    z = z + l_4 - d_b
  
    d = math.sqrt(x**2 + y**2)
    if d > 614.5+l_4:
        return None, float('inf')  # Position out of bounds, infinite cost

    c2 = (x**2 + y**2 - (l_1**2 + l_2**2)) / (2 * l_1 * l_2)
    s2_v1 = -math.sqrt(1 - c2**2)
    s2_v2 = math.sqrt(1 - c2**2)

    theta_2_v1 = math.atan2(s2_v1, c2)
    theta_2_v2 = math.atan2(s2_v2, c2)

    theta_1_v1 = math.atan2(x, y) - math.atan2(l_2 * s2_v1, l_1 + l_2 * c2)
    theta_1_v2 = math.atan2(x, y) - math.atan2(l_2 * s2_v2, l_1 + l_2 * c2)

    theta_1_v1_deg = math.degrees(theta_1_v1)
    theta_2_v1_deg = math.degrees(theta_2_v1)

    theta_1_v2_deg = math.degrees(theta_1_v2)
    theta_2_v2_deg = math.degrees(theta_2_v2)

    solutions = []
    # print("THETA1 v1:", theta_1_v1_deg)
    # print("THETA2 v1:", theta_2_v1_deg)

    # print("THETA1 v2:", theta_1_v2_deg)
    # print("THETA2 v2:", theta_2_v2_deg)

    if -150 < theta_2_v1_deg < 150:
        solutions.append((theta_1_v1_deg, theta_2_v1_deg))

    if -150 < theta_2_v2_deg < 150:
        solutions.append((theta_1_v2_deg, theta_2_v2_deg))

    # Calculate cost based on joint movement and differences from pos
    min_cost = float('inf')
    best_solution = None

    for solution in solutions:
        cost = abs(solution[0] - pos[0]) + abs(solution[1] - pos[2])
        if cost < min_cost or (cost == min_cost and solution[0] < best_solution[0]):
            min_cost = cost
            best_solution = solution

    theta_1 = round(best_solution[0], 2)  # Use best_solution instead of solution
    d = round(z, 2)
    theta_3 = round(best_solution[1], 2)  # Use best_solution instead of solution
    
    # alpha = theta_1 + theta_3 + theta_4
    theta_4 = gripper_or - theta_1 - theta_3
    theta_4 = round(theta_4, 2)


    # print(f"Best Solution: theta_1: {theta_1}, d: {d}, theta_3: {theta_3}, theta_4: {theta_4}")
    return theta_1, d, theta_3, theta_4



# # Example usage:
# x_input = float(input("Enter the value for x: "))
# y_input = float(input("Enter the value for y: "))
# z_input = float(input("Enter the value for z: "))
# a_input = float(input("Enter the value for gripper orientation: "))
# speed_input = float(input("Enter the speed: "))
# pos_input = [float(input("Enter the current value for j1: ")),
#              float(input("Enter the current value for j2: ")),
#              float(input("Enter the current value for j3: ")),
#              float(input("Enter the current value for j4: "))]

# theta_1, d, theta_3, theta_4 = calculate_scara_angles(x_input, y_input, z_input, a_input, pos_input)

# print("THETA_1: ", theta_1)
# print("D: ", d)
# print("THETA_3: ", theta_3)
# print("THETA_4: ", theta_4)


# theta_1, d_1, theta_3, theta_4 = calculate_scara_angles(-500, -300, 300, -90, [400, 400, 200, 0])
# print("THETA1:", theta_1)
# print("D2:", d_1)
# print("THETA3:", theta_3)
# print("THETA4:", theta_4)


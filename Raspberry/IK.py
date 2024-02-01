import math

def calculate_scara_angles(x, y, z, gripper_or, pos):
    l_1 = 329.5 #First arm link
    l_2 = 285.0 #Second arm link
    d_b = 123.0 #Base offset

    #Change according to desired TCP
    l_3 = 25.65 # TCP vertical offset
    l_4 = 102.0 # TCP horizontal offset
    # l_3 = 110.0 # TCP vertical offset

    gripper_or_radians = math.radians(gripper_or)
    
    #From TCP to J4
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

# First solution
    theta_1_v1 = math.atan2(x, y) - math.atan2(l_2 * s2_v1, l_1 + l_2 * c2)
    theta_2_v1 = math.atan2(s2_v1, c2)
    theta_1_v1_deg = math.degrees(theta_1_v1)
    theta_2_v1_deg = math.degrees(theta_2_v1)

#Second solution
    theta_1_v2 = math.atan2(x, y) - math.atan2(l_2 * s2_v2, l_1 + l_2 * c2)
    theta_2_v2 = math.atan2(s2_v2, c2)
    theta_1_v2_deg = math.degrees(theta_1_v2)
    theta_2_v2_deg = math.degrees(theta_2_v2)

    solutions = []

#Check if J3 fits within range
    if -150 < theta_2_v1_deg < 150:
        solutions.append((theta_1_v1_deg, theta_2_v1_deg))

    if -150 < theta_2_v2_deg < 150:
        solutions.append((theta_1_v2_deg, theta_2_v2_deg))

    # Calculate cost based on joint movement and differences from pos
    min_cost = float('inf')
    best_solution = None

    for solution in solutions:
        cost = abs(solution[0] - pos[0]) + abs(solution[1] - pos[2])
        #Choose solution (currently IK chooses the left position if solutions have the same cost)
        if cost < min_cost or (cost == min_cost and solution[0] < best_solution[0]):
            min_cost = cost
            best_solution = solution

    d = round(z, 2)
    theta_1 = round(best_solution[0], 2)
    theta_3 = round(best_solution[1], 2)

    theta_4 = gripper_or - theta_1 - theta_3
    theta_4 = round(theta_4, 2)


    # print(f"Best Solution: theta_1: {theta_1}, d: {d}, theta_3: {theta_3}, theta_4: {theta_4}")
    return theta_1, d, theta_3, theta_4





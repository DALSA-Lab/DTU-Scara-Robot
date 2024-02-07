import serial
import time
import ast
import math
import IK
from decimal import Decimal

#Read last stored position
file_path = "pos_rec.txt"
def read_stored_positions(file_path):
    with open(file_path, "r") as pos_file:
        lines = pos_file.readlines()
        if lines:
            last_line = lines[-1].strip()
        else:
            # Return default values if the file is empty
            return [0.0, 0.0, 0.0, 0.0]

    stored_positions = ast.literal_eval(last_line)

    return stored_positions

#Position prediction
def track(response, j_diff, target):
    stored_positions = read_stored_positions(file_path)
    pos_counter = 0
    prev_difference = []
    j_diff_absolute = [stored + diff for stored, diff in zip(stored_positions, j_diff)]

    while True:
        ser.write('M114 R\n'.encode())
        response = ser.readline().decode()
        if 'X:' not in response or 'Y:' not in response or 'Z:' not in response or 'A:' not in response:
            continue
        else:
            position_data = response.split('Count')[0].strip()
            position_items = [item.split(':') for item in position_data.split()]

            positions_list = []
            for item in position_items:
                if len(item) == 2:
                    letter, value = item
                    positions_list.append(float(Decimal(value.strip())))
                else:
                    # If the item does not have 2 elements, ignore it
                    continue
                    
            received_positions = positions_list


            stored_positions = received_positions
            with open(file_path, "w") as pos_file:
                pos_file.write(f"{received_positions}\n")
            
            print(" ")
            print("rec_pos:", received_positions)
            print("STORED: ", stored_positions)
            print("+")
            print("J_DIFF: ", j_diff)
            print("J DIFF ABS:", j_diff_absolute)
            
                       
            if all(math.isclose(float(p1), p2, abs_tol=1e-2) for p1, p2 in zip(received_positions, j_diff_absolute)):
                pos_counter += 1
            else:
                pos_counter = 0
                prev_pos = received_positions
            if pos_counter == 5:
                print("ARRIVED")
                break
            
        

# MAKE SURE THESE VALUES ARE THE SAME AS IN HOMING.PY
zero_positions = [0.0, 330.0, -150.0, 0.0]

j1_home = False
j2_home = False
j3_home = False
j4_home = False

#Open serial com
ser = serial.Serial('/dev/ttyUSB0', baudrate=250000, timeout=0.02)

#Receive all initial Marlin responses
ser.write('M115\n'.encode())

end_time = time.time() + 1
while time.time() < end_time:
    while ser.in_waiting > 0:
        response = ser.readline().decode()

ser.write('M119\n'.encode())
response = ""
while True:
	line = ser.readline().decode()
	if not line:
		break
	response += line
	print(line.strip())

# Check endstop states
for line in response.split('\n'):
    if 'x_min' in line and 'TRIGGERED' in line:
        j1_home = True

    elif 'y_max' in line and 'TRIGGERED' in line:
        j2_home = True

    elif 'z_min' in line and 'TRIGGERED' in line:
        j3_home = True

    elif 'a_min' in line and 'TRIGGERED' in line:
        j4_home = True


if j1_home and j2_home and j3_home and j4_home:
    print("All desired endstops are homed. Entering the main loop.")
    #Custom command
    ser.write(f'G92 X{zero_positions[0]} Y{zero_positions[1]} Z{zero_positions[2]} A{zero_positions[3]}\n'.encode())


    try:
        while True:
            stored_pos = read_stored_positions(file_path)
            x = float(input("Enter x position: "))
            y = float(input("Enter y position: "))
            z = float(input("Enter z position: "))
            a = float(input("Enter gripper orientation: "))
            
            target = [x, y, z, a]
            
            # Get user input for speed and acceleration
            speed = float(input("Enter speed: "))
            acceleration = float(input("Enter acceleration: "))

            # Calculate inverse kinematics to obtain Cartesian coordinates
            j1, j2, j3, j4 = IK.calculate_scara_angles(x, y, z, a, stored_pos)
            print("CALCULATED J:")
            print("j1:", j1)
            print("j2:", j2)
            print("j3:", j3)
            print("j4:", j4)
            
            print("stored j1:", stored_pos[0])
            print("stored j2:", stored_pos[1])
            print("stored j3:", stored_pos[2])
            print("stored j4:", stored_pos[3])
            
            stored_pos = read_stored_positions(file_path)
            j_diff = [0, 0, 0, 0]
            j_diff[0] = j1 - stored_pos[0]
            j_diff[1] = j2 - stored_pos[1] 
            j_diff[2] = j3 - stored_pos[2]
            j_diff[3] = j4 - stored_pos[3]
            
            j_diff = [0 if abs(elem) < 0.01 else elem for elem in j_diff]
            
            print("J_DIFF:", j_diff)
            ser.write('G91\n'.encode())
            response = ser.readline().decode()

            acc_command = f"M204 S{acceleration}"
            #Custom command
            command = f"G1 X{j_diff[0]} Y{j_diff[1]} Z{j_diff[2]} A{j_diff[3]} F{speed}"
            print("CMD:", command)

            # Send command to the MKS board
            ser.write((acc_command + '\n').encode())
            response = ser.readline().decode()
            time.sleep(0.1)
            
            time.sleep(3)
            ser.write((command + '\n').encode())

            response = ser.read(1024).decode()
            
            test = track(response, j_diff, target)

    except KeyboardInterrupt:
        ser.close()
        print("\nSerial port closed. Exiting...")
else:
    print("Not all desired endstops are homed. Please home all endstops before entering the main loop.")
    print("j1:", j1_home)
    print("j2:", j2_home)
    print("j3:", j3_home)
    print("j4:", j4_home)

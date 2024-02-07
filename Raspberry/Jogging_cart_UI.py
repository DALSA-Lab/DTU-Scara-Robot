import serial
import time
import ast
import math
import IK
from decimal import Decimal
import argparse

file_path = "pos_rec.txt"
def read_stored_positions(file_path):
    with open(file_path, "r") as pos_file:
        lines = pos_file.readlines()
        if lines:
            last_line = lines[-1].strip()
        else:
            # Return default values if the file is empty
            return [0.0, 0.0, 0.0, 0.0]

    # Safely evaluate the content as a Python literal (list in this case)
    stored_positions = ast.literal_eval(last_line)

    return stored_positions

def track(j_diff, stored_pos, ser):

    pos_counter = 0
    goal_pos = [stored + diff for stored, diff in zip(stored_pos, j_diff)]
    
    ser.write('G91\n'.encode())

    while True:
        # Position prediction
        ser.write('M114 R\n'.encode())
        response = ser.readline().decode()
        if 'X:' not in response or 'Y:' not in response or 'Z:' not in response or 'A:' not in response:
            continue
        else:
            position_data = response.split('Count')[0].strip()
            position_items = [item.split(':') for item in position_data.split()]
            print("RECEIVED POSITIONS: ", response)

            # Create a list to store decimal positions
            positions_list = []
            for item in position_items:
                if len(item) == 2:
                    letter, value = item
                    positions_list.append(float(Decimal(value.strip())))
                else:
                    # If the item does not have 2 elements, ignore it
                    continue

            received_positions = positions_list

            with open(file_path, "w") as pos_file:
                pos_file.write(f"{received_positions}\n")
            
            print(" ")
            print("rec_pos:", received_positions)
            print("Goal position", goal_pos)
            
            # Check if last 5 tracked values are the same          
            if all(math.isclose(float(p1), p2, abs_tol=1e-2) for p1, p2 in zip(received_positions, goal_pos)):
                pos_counter += 1
            else:
                pos_counter = 0
                prev_pos = received_positions
            if pos_counter == 5:
                print("ARRIVED")
                break
            
        
def main(x, y, z, a, speed, acceleration):
    
    # Initial endstop condition
    j1_home = False
    j2_home = False
    j3_home = False
    j4_home = False

    ser = serial.Serial('/dev/ttyS0', baudrate=250000, timeout=0.02)

    #Receive all initial responses from Marlin
    ser.write('M115\n'.encode())
    end_time = time.time() + 1
    while time.time() < end_time:
        while ser.in_waiting > 0:
            response = ser.readline().decode()

	# Overwrite coordinate system with the last stored values
    stored_pos = read_stored_positions(file_path)
    load_coords_cmd = f"G92 X{stored_pos[0]} Y{stored_pos[1]} Z{stored_pos[2]} A{stored_pos[3]}"
    ser.write((load_coords_cmd + '\n').encode())
    response = ser.readline().decode()
    target = [x, y, z, a]

    # Joint calculations
    j1, j2, j3, j4 = IK.calculate_scara_angles(x, y, z, a, stored_pos) #calculate joints from IK
    j_diff = [j1 - stored_pos[0], j2 - stored_pos[1], j3 - stored_pos[2], j4 - stored_pos[3]] #calculate joint movement
    j_diff = [0 if abs(elem) < 0.01 else elem for elem in j_diff]
    
    print("J_DIFF:", j_diff)

    # Sending commands
    ser.write('G91\n'.encode())
    response = ser.readline().decode()

    acc_command = f"M204 S{acceleration}"
    # Send custom command
    command = f"G1 X{j_diff[0]} Y{j_diff[1]} Z{j_diff[2]} A{j_diff[3]} F{speed}" #write joint movement cmd
    print("CMD:", command)

    # Send command to the MKS board
    ser.write((acc_command + '\n').encode())
    response = ser.read(1024).decode()
    time.sleep(0.1)

    time.sleep(3)
    ser.write((command + '\n').encode())

    # Read responses
    response = ser.read(1024).decode()
    track(j_diff, stored_pos, ser)

#Variable parsing
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send IK command.")
    parser.add_argument("--x", type=float, required=True, help="X position")
    parser.add_argument("--y", type=float, required=True, help="Y position")
    parser.add_argument("--z", type=float, required=True, help="Z position")
    parser.add_argument("--a", type=float, required=True, help="A (Gripper Orientation)")
    parser.add_argument("--speed", type=float, required=True, help="Speed")
    parser.add_argument("--acceleration", type=float, required=True, help="Acceleration")

    args = parser.parse_args()
    main(args.x, args.y, args.z, args.a, args.speed, args.acceleration)

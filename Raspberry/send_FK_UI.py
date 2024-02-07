import serial
import time
import ast
import math
import IK
from decimal import Decimal
import argparse

#Read stored positions
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

def track(goal_pos, stored_pos, ser):

    pos_counter = 0

    while True:
        # Position prediction
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
                    continue

            received_positions = positions_list

            with open(file_path, "w") as pos_file:
                pos_file.write(f"{received_positions}\n")
            
            
            # Check if last 5 tracked values are the same          
            if all(math.isclose(float(p1), p2, abs_tol=1e-2) for p1, p2 in zip(received_positions, goal_pos)):
                pos_counter += 1
            else:
                pos_counter = 0
                prev_pos = received_positions
            if pos_counter == 5:
                print("ARRIVED")
                break
            
        
def main(j1, j2, j3, j4, speed, acceleration):
    
    # Initial endstop condition
    j1_home = False
    j2_home = False
    j3_home = False
    j4_home = False

    #Open serial com
    ser = serial.Serial('/dev/ttyS0', baudrate=250000, timeout=0.02)

    # Receive and print all available responses from Marlin with timestamps
    ser.write('M115\n'.encode())
    end_time = time.time() + 1
    while time.time() < end_time:
        while ser.in_waiting > 0:
            response = ser.readline().decode()

    stored_pos = read_stored_positions(file_path)

    goal_pos = [j1, j2, j3, j4]

    # Change commands to absolute coordinates
    ser.write('G90\n'.encode())
    response = ser.readline().decode()

    acc_command = f"M204 S{acceleration}"
    # Custom command string
    command = f"G0 X{goal_pos[0]} Y{goal_pos[1]} Z{goal_pos[2]} A{goal_pos[3]} F{speed}" #write joint movement cmd
    print("CMD:", command)

    # Send command to the MKS board
    ser.write((acc_command + '\n').encode())
    response = ser.read(1024).decode()
    time.sleep(0.1)

    ser.write((command + '\n').encode())


    response = ser.read(1024).decode()
    track(goal_pos, stored_pos, ser)


#Variables parsing
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send FK command.")
    parser.add_argument("--J1", type=float, required=True, help="J1 position")
    parser.add_argument("--J2", type=float, required=True, help="J2 position")
    parser.add_argument("--J3", type=float, required=True, help="J3 position")
    parser.add_argument("--J4", type=float, required=True, help="J4 (Gripper Orientation)")
    parser.add_argument("--speed", type=float, required=True, help="Speed")
    parser.add_argument("--acceleration", type=float, required=True, help="Acceleration")

    args = parser.parse_args()
    main(args.J1, args.J2, args.J3, args.J4, args.speed, args.acceleration)

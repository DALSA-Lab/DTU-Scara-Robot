import time
import serial
import ast
from decimal import Decimal

file_path = "pos_rec.txt"

#Read last stored values from txt
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

def extract_positions(response):
    # Extract X, Y, Z, A positions from the response
    position_data = response.split('Count')[0].strip()
    position_items = [item.split(':') for item in position_data.split()]

    # Create a list to store decimal positions
    positions_list = []
    for item in position_items:
        if len(item) == 2:
            letter, value = item
            # Convert the value to Decimal, then to float, and append to the list
            positions_list.append(float(Decimal(value.strip())))
        else:
            # If the item does not have 2 elements, ignore it
            continue

    # Return the positions as a list of decimals
    return positions_list

def homing_check(ser, endstop, stored_pos):
    # Read the stored positions
    stored_positions = stored_pos
    endstop_mapping = {'x_min': 0, 'y_min': 1, 'y_max': 1, 'z_min': 2, 'z_max': 2, 'a_min': 3}
    prev_difference_pos = None
    stop_command = f'M410\n'

    while True:
        # Send command to get current position
        ser.write('M114 R\n'.encode())
        response = ser.readline().decode()
        if 'X:' not in response or 'Y:' not in response or 'Z:' not in response or 'A:' not in response:
            continue
        else:
            # Extract and record positions to the text file
            change_positions = extract_positions(response)

            # Calculate the difference and write to the file
            if change_positions != '[]':
                difference_pos = [stored + change if stored >= 0 else stored + change for change, stored in zip(change_positions, stored_positions)]
            with open(file_path, "w") as pos_file:  # Use "w" mode to overwrite the file
                pos_file.write(f"{difference_pos}\n")

            print("stored_pos:", stored_positions)
            print("rec_pos:", change_positions)
            print("difference_pos:", difference_pos)

        ser.write('M119\n'.encode())
        response = ""
        while True:
            line = ser.readline().decode()
            if not line:
                break  # Exit the loop when no more lines are available
            response += line
            print(line.strip())  # Print each line

        endstop_status = None
        for line in response.split('\n'):
            if endstop in line:
                endstop_status = line.split(':')[-1].strip()
                break

        # Status check
        if endstop_status == 'TRIGGERED':
            # Stop motor
            ser.write(stop_command.encode())
            # Set real pos                      
            zero_positions = stored_pos
            if endstop == 'x_min':
                zero_positions[0] = -1.2
                ser.write(f'G92 X{zero_positions[0]}\n'.encode())
            elif endstop == 'y_min':
                zero_positions[1] = 0.0
                ser.write(f'G92 Y{zero_positions[1]}\n'.encode())
            elif endstop == 'y_max':
                zero_positions[1] = 330.0
                ser.write(f'G92 Y{zero_positions[1]}\n'.encode())
            elif endstop == 'z_min':
                zero_positions[2] = -149.625
                ser.write(f'G92 Z{zero_positions[2]}\n'.encode())
            elif endstop == 'z_max':
                zero_positions[2] = 149.625
                ser.write(f'G92 Z{zero_positions[2]}\n'.encode())
            elif endstop == 'a_min':
                zero_positions[3] = 0.0
                ser.write(f'G92 A{zero_positions[3]}\n'.encode())

            endstop = endstop[0]
            time.sleep(0.05)
            print("TRIGGERED")
            

            with open(file_path, "w") as pos_file:
                pos_file.write(f"{zero_positions}\n")
                break
        
        #Check if the position matches the stored position
        elif endstop_status == 'open' and prev_difference_pos is not None and difference_pos[endstop_mapping[endstop]] == prev_difference_pos[endstop_mapping[endstop]]:
            print("VAL:", difference_pos[endstop_mapping[endstop]])
            print("PREV VAL:", prev_difference_pos[endstop_mapping[endstop]])
            ser.write(stop_command.encode())
            print("HOMING FAILED")
            break
        prev_difference_pos = difference_pos


        time.sleep(0.01)

specified_endstops = ['x_min', 'y_max', 'z_min', 'a_min']

# Start serial
ser = serial.Serial('/dev/ttyS0', baudrate=250000, timeout=0.2)
time.sleep(1)

ser.write('M115\n'.encode())

# Loop that lasts for 1 second
end_time = time.time() + 1
while time.time() < end_time:
    # Receive and print all available responses from Marlin with timestamps
    while ser.in_waiting > 0:
        response = ser.readline().decode()
        

time.sleep(3)
ser.write('G90\n'.encode())
response = ser.readline().decode()
print(response)
time.sleep(0.05)
ser.write('M204 S15\n'.encode())
response = ser.readline().decode()
print(response)
time.sleep(0.05)



#HOMING SEQUENCE
# A homing
stored_pos = read_stored_positions(file_path)
print("J4 HOMING")
ser.write(f'G0 A0 F2000\n'.encode())
homing_check(ser, 'a_min', stored_pos)

time.sleep(0.1)


# Y offset
ser.write('G91\n'.encode())
response = ser.readline().decode()
print(response)
stored_pos = read_stored_positions(file_path)
ser.write(f'G1 Y100 F1000\n'.encode())
homing_check(ser, 'y_max', stored_pos)
ser.write('G90\n'.encode())
response = ser.readline().decode()
print(response)
time.sleep(0.5)


# Z homing
stored_pos = read_stored_positions(file_path)
print("J3 HOMING")
# Adding a error margin
ser.write(f'G0 Z-155 F2000\n'.encode())
homing_check(ser, 'z_min', stored_pos)
print("HOME")
time.sleep(0.1)


# Y homing
ser.write('G91\n'.encode())
response = ser.readline().decode()
print(response)
stored_pos = read_stored_positions(file_path)
print("J2 HOMING")
ser.write(f'G1 Y250 F1000\n'.encode())
homing_check(ser, 'y_max', stored_pos)
print("HOME")
ser.write('G90\n'.encode())
response = ser.readline().decode()
print(response)
time.sleep(0.1)


# X homing
stored_pos = read_stored_positions(file_path)
print("J1 HOMING")
ser.write(f'G0 X0 F500\n'.encode())
homing_check(ser, 'x_min', stored_pos)


    
# # Check the status of specified endstops
# failed_endstops = [endstop for endstop in specified_endstops if f'{endstop}: TRIGGERED' not in response]

# if not failed_endstops:
#     print("HOMING COMPLETED")
#     coord_command = f'G92 X{x_min_pos} Y{y_max_pos} Z{z_min_pos} A{a_min_pos}\n'
#     ser.write(coord_command.encode())
# else:
#     for endstop in failed_endstops:
#         if endstop == 'x_min':
#             print("HOMING FAILED FOR J1")
#         elif endstop == 'y_max':
#             print("HOMING FAILED FOR J2")
#         elif endstop == 'z_min':
#             print("HOMING FAILED FOR J3")
#         elif endstop == 'a_min':
#             print("HOMING FAILED FOR J4")
    
    
ser.close()


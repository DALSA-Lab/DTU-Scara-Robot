import time
import serial

file_path = "pos_rec.txt"
zero_positions = [None, None, None, None]
x_min_pos = -1.2
y_min_pos = 0.0
y_max_pos = 0.0
z_min_pos = -150.0
z_max_pos = 150.0
a_min_pos = 0.0

def homing_check(ser, endstop):
    start_time = time.time()
    stop_command = f'M410\n'
    
    #Check endstop status
    while True:
        ser.write('M119\n'.encode())
        response = ""
        while True:
            line = ser.readline().decode()
            if not line:
                break
            response += line
            print(line.strip())   

        endstop_status = None
        for line in response.split('\n'):
            if endstop in line:
                endstop_status = line.split(':')[-1].strip()
                break

        # Status check
        if endstop_status == 'TRIGGERED':
            # Stop motor
            ser.write(stop_command.encode())

            print("ENDSTOP:", endstop)
            # Set real pos
            if endstop == 'x_min':
                zero_positions[0] = x_min_pos
                print("X_MIN ZERO:", x_min_pos)
                set_zero = f'G92 X{x_min_pos}\n'
                ser.write(set_zero.encode())
            elif endstop == 'y_min':
                zero_positions[1] = y_min_pos
                print("Y_MIN ZERO:", y_min_pos)
                set_zero = f'G92 Y{y_min_pos}\n'
                ser.write(set_zero.encode())
            elif endstop == 'y_max':
                zero_positions[1] = y_max_pos
                print("Y_MAX ZERO:", y_max_pos)
                set_zero = f'G92 Y{y_max_pos}\n'
                ser.write(set_zero.encode())
            elif endstop == 'z_min':
                zero_positions[2] = z_min_pos
                print("Z_MIN ZERO:", z_min_pos)
                set_zero = f'G92 Z{z_min_pos}\n'
                ser.write(set_zero.encode())
            elif endstop == 'z_max':
                zero_positions[2] = z_max_pos
                print("Z_MAX ZERO:", z_max_pos)
                set_zero = f'G92 Z{z_max_pos}\n'
                ser.write(set_zero.encode())
            elif endstop == 'a_min':
                zero_positions[3] = a_min_pos
                print("A_MIN ZERO:", a_min_pos)
                set_zero = f'G92 A{a_min_pos}\n'
                ser.write(set_zero.encode())
            
            endstop = endstop[0]
            time.sleep(0.05)
            print("TRIGGERED")
            
            # Set real zero
            set_zero = f'G92 {endstop.capitalize()}0\n'
            ser.write(set_zero.encode())
            time.sleep(0.01)
            
            with open(file_path, "w") as pos_file:
                print("WRITING ZERO POSITIONS:", zero_positions)
                pos_file.write(f"{zero_positions}\n")
                break

        elif endstop_status == 'open' and (time.time() - start_time >= 5) and (endstop == 'x_min' or endstop == 'a_min'):
            break

        time.sleep(0.01)

# Start serial
ser = serial.Serial('/dev/ttyS0', baudrate=250000, timeout=0.2)
time.sleep(1)

ser.write('M115\n'.encode())

# Receive and print all available responses from Marlin
end_time = time.time() + 1
while time.time() < end_time:

    while ser.in_waiting > 0:
        response = ser.readline().decode()
        

time.sleep(3)
ser.write('G91\n'.encode())
response = ser.readline().decode()
print(response)
time.sleep(0.05)
ser.write('M204 S15\n'.encode())
response = ser.readline().decode()
print(response)
time.sleep(0.05)


#HOMING SEQUENCE
# Y offset
print("Y OFFSET")
ser.write('G1 Y100 F1000\n'.encode())
time.sleep(5)

# A homing
print("A HOMING")
ser.write('G1 A15 F750\n'.encode())
homing_check(ser, 'a_min')
time.sleep(0.1)
ser.write('G1 A-30 F750\n'.encode())
homing_check(ser, 'a_min')
time.sleep(0.1)

# Z homing
print("Z HOMING")
ser.write('G1 Z-305 F1000\n'.encode())
homing_check(ser, 'z_min')
time.sleep(0.1)

# Y homing
print("Y HOMING")
ser.write('G1 Y255 F1000\n'.encode())
homing_check(ser, 'y_max')
time.sleep(0.1)

# X homing
print("X HOMING")
ser.write('G1 X10 F350\n'.encode())
homing_check(ser, 'x_min')
time.sleep(0.1)
ser.write('G1 X-20 F350\n'.encode())
homing_check(ser, 'x_min')
time.sleep(0.1)

time.sleep(3)

# Get endstop status
ser.write('M119\n'.encode())
response = ""
while True:
    line = ser.readline().decode()
    if not line:
        break
    response += line
    print(line.strip())

# List of specified endstops, change accoring to used endstop
specified_endstops = ['x_min', 'y_max', 'z_min', 'a_min']

# Check the status of specified endstops
failed_endstops = [endstop for endstop in specified_endstops if f'{endstop}: TRIGGERED' not in response]

if not failed_endstops:
    print("HOMING COMPLETED")
    coord_command = f'G92 X{x_min_pos} Y{y_max_pos} Z{z_min_pos} A{a_min_pos}\n'
    ser.write(coord_command.encode())
else:
    for endstop in failed_endstops:
        if endstop == 'x_min':
            print("HOMING FAILED FOR J1")
        elif endstop == 'y_max':
            print("HOMING FAILED FOR J2")
        elif endstop == 'z_min':
            print("HOMING FAILED FOR J3")
        elif endstop == 'a_min':
            print("HOMING FAILED FOR J4")

    
ser.close()


import serial
import time

#Open serial com
ser = serial.Serial('/dev/ttyUSB0', baudrate=250000, timeout=1)

# Read and send the contents of the gcode.txt file
with open('gcode.txt', 'r') as file:
    for line in file:
        ser.write((line.strip() + '\n').encode())
        time.sleep(0.2)

        response = ser.readline().decode()
        print(response, end='')

# Close the serial port
ser.close()
print("\nSerial port closed. Exiting...")

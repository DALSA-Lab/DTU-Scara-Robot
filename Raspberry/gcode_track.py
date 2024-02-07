import serial
import time
import signal
import sys

def signal_handler(sig, frame):
    print("\nCtrl+C detected. Exiting...")
    ser.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#Open serial com
ser = serial.Serial('/dev/ttyUSB0', baudrate=250000, timeout=0.002)

try:
    # Ping Marlin when the code is initiated
    ser.write('M115\n'.encode())
    time.sleep(0.2)
    end_time = time.time() + 1
    while time.time() < end_time:
        # Receive and print all available responses from Marlin
        while ser.in_waiting > 0:
            response = ser.readline().decode()

    # Read and send the contents of the gcode.txt file
    with open('gcode.txt', 'r') as file:
        for line in file:
            ser.write((line.strip() + '\n').encode())
            time.sleep(0.002)  # Add a 2ms delay after sending each line

    # Position prediction
    while True:
        ser.write('M114 R\n'.encode())
        response = ser.readline().decode()
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) + f".{int((time.time() % 1) * 1000):03d}"
        print(f"[{timestamp}] {response}", end='')
        time.sleep(0.002)

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Close the serial port
    time.sleep(60)
    ser.close()
    print("Serial port closed. Exiting...")

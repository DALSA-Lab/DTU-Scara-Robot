import time
import serial


# Start serial
ser = serial.Serial('/dev/ttyS0', baudrate=250000, timeout=0.2)
time.sleep(1)
ser.write('M410\n'.encode())

# Receive and print all available responses from Marlin
end_time = time.time() + 1
while time.time() < end_time:
    while ser.in_waiting > 0:
        response = ser.readline().decode()
        
    
ser.close()


import serial

#Open serial com
ser = serial.Serial('/dev/ttyS0', baudrate=250000, timeout=0.2)


try:
    while True:
        # Get user input
        command = input("Enter G-code command (e.g., G1 X10 F500): ")

        # Send command to the MKS board
        ser.write((command + '\n').encode())

        response = ser.read(1024).decode()
        print(response, end='')

except KeyboardInterrupt:
    ser.close()
    print("\nSerial port closed. Exiting...")

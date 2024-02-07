# import serial
import argparse



#Send move command
def send_cmd(joint1, joint2, joint3, joint4, speed, acc):
    ser = serial.Serial('/dev/ttyS0', baudrate=250000, timeout=0.2)

    acc_cmd = f'M205 S{acc}'
    ser.write((acc_cmd + '\n').encode())

    gcode_cmd = f'G0 X{joint1} Y{joint2} Z{joint3} A{joint4} F{speed}'

    ser.write((gcode_cmd + '\n').encode())
    ser.close()


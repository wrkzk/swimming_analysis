# Script to interface with a serial device and capture data
# Writes all lines between '---FILE START---' and '---FILE END---'
# to a file named 'data.csv' in the current directory

import serial

ser = serial.Serial('COM12', 115200)
with open('data.csv', 'w') as f:
    capturing = False
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if '---FILE START---' in line:
            capturing = True
            continue
        elif '---FILE END---' in line:
            break;
        if capturing:
            f.write(line + '\n')
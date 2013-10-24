import serial
import time

ser = serial.Serial("/dev/ttyACM0", timeout=0)
lines = []

while(True):
    lines.extend(ser.readlines())
    if len(lines) > 0 and lines[-1][-1] == '\n':
        print "\n".join(lines)
        lines = []
    time.sleep(0.1)


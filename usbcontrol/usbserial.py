#!/usr/bin/env python

# requires pyserial
import serial
import sys

# stdin from usbserial
# writes to arduino

def main():
    print sys.argv[1]
    ser = serial.Serial(sys.argv[1])
    while True:
        cmd = raw_input()
        axis, value = cmd[0], int(cmd[1:])
        # value is -127..127
        value_ = value + 127 # adjust to 0..254
        out = '%s%s' % (axis, chr(value_))
        ser.write(out)

if __name__ == '__main__':
    main()

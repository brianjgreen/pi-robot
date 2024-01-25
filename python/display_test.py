#!/usr/bin/python

import smbus
import time

bus = smbus.SMBus(1)

DEVICE_ADDRESS = 0x71
DEVICE_REG_MODE1 = 0x00
DEVICE_REG_PRESCALE = 0xFE
DEVICE_REG_LEDOUT0 = 0x1D

D = {
    ' ': 0x00,
    '-': 0x40,
    '0': 0x3F,
    '1': 0x06,
    '2': 0x5B,
    '3': 0x4F,
    '4': 0x66,
    '5': 0x6D,
    '6': 0x7D,
    '7': 0x07,
    '8': 0x7F,
    '9': 0x6F,
    'A': 0x77,
    'B': 0x7C,
    'C': 0x39,
    'D': 0x5E,
    'E': 0x79,
    'F': 0x71,
    'H': 0x76,
    'L': 0x38
}

# Hold in reset
bus.write_byte(DEVICE_ADDRESS, 0x21)
# Prescale 0x03 is about 1600 Hz
bus.write_byte(DEVICE_ADDRESS, 0x81)
# Come out of reset, enable auto register address increment and run!
bus.write_byte(DEVICE_ADDRESS, 0xE8)

time.sleep(0.005)  # 5ms (requires 500 us)

display = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
def display_str(message):
    i = 0;
    for c in message:
        if c == '.':
            display[i-2] = display[i-2] + 0x80
            continue
        if i < 16:
            display[i] = D[c]
        i = i + 2
        if i == 4:
            i = 6
    bus.write_i2c_block_data(DEVICE_ADDRESS, 0x00, display)

def scroll_str(message):
    scroll = '   ' + message + '   '
    i = 0
    for c in scroll:
        display_str(scroll[i:i+4])
        time.sleep(0.25)
        i = i + 1
        
while 1:
    scroll_str('HELL0')
    #display_str('1.28')
    #time.sleep(3)
    display_str('H 77')
    time.sleep(3)
    display_str('L 56')
    time.sleep(3)

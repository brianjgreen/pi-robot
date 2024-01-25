#!/usr/bin/python

import smbus
import time

bus = smbus.SMBus(1)

DEVICE_ADDRESS = 0x60
DEVICE_REG_MODE1 = 0x00
DEVICE_REG_PRESCALE = 0xFE
DEVICE_REG_LEDOUT0 = 0x1D

# Hold in reset
bus.write_byte_data(DEVICE_ADDRESS, DEVICE_REG_MODE1, 0x10)
# Prescale 0x03 is about 1600 Hz
bus.write_byte_data(DEVICE_ADDRESS, DEVICE_REG_PRESCALE, 0x03)
# Come out of reset, enable auto register address increment and run!
bus.write_byte_data(DEVICE_ADDRESS, DEVICE_REG_MODE1, 0xA0)

time.sleep(0.005)  # 5ms (requires 500 us)

# Motor to PWM pin mappings
motor = [
    {'pwm' : 0x26, 'in1' : 0x2e, 'in2' : 0x2a},  # motor 1
    {'pwm' : 0x3a, 'in1' : 0x32, 'in2' : 0x36},  # motor 2
    {'pwm' : 0x0e, 'in1' : 0x16, 'in2' : 0x12},  # motor 3
    {'pwm' : 0x22, 'in1' : 0x1a, 'in2' : 0x1e}   # motor 4
]

# Speed
speed = {
    'high' : [0x00, 0x10, 0x00, 0x00],
    'speed25' : [0x00, 0x00, 0x00, 0x04],
    'speed50' : [0x00, 0x00, 0x00, 0x08],
    'speed75' : [0x00, 0x00, 0x00, 0x0C],
    # speed100 is the same as 'high'
    # stop is the same as 'low'
    'low' : [0x00, 0x00, 0x00, 0x10]
}

# Always write the LOW pin first
# Motor will do a "short break" if both in1 and in2 are high
def direction(dir, in1, in2):
    if dir == 'forward':
        bus.write_i2c_block_data(DEVICE_ADDRESS, in2, speed['low'])
        bus.write_i2c_block_data(DEVICE_ADDRESS, in1, speed['high'])
    elif dir == 'reverse':
        bus.write_i2c_block_data(DEVICE_ADDRESS, in1, speed['low'])
        bus.write_i2c_block_data(DEVICE_ADDRESS, in2, speed['high'])
    else:
        # release (stop)
        bus.write_i2c_block_data(DEVICE_ADDRESS, in1, speed['low'])
        bus.write_i2c_block_data(DEVICE_ADDRESS, in2, speed['low'])

def velocity(dir, speed):
    for i in motor:
        bus.write_i2c_block_data(DEVICE_ADDRESS, i['pwm'], speed)
    if dir == 'forward':
        for i in motor:
            direction('forward', i['in1'], i['in2'])
    elif dir == 'reverse':
        for i in motor:
            direction('reverse', i['in1'], i['in2'])
    elif dir == 'right':
        direction('reverse', motor[0]['in1'], motor[0]['in2'])
        direction('reverse', motor[1]['in1'], motor[1]['in2'])
        direction('forward', motor[2]['in1'], motor[2]['in2'])
        direction('forward', motor[3]['in1'], motor[3]['in2'])
    elif dir == 'left':
        direction('forward', motor[0]['in1'], motor[0]['in2'])
        direction('forward', motor[1]['in1'], motor[1]['in2'])
        direction('reverse', motor[2]['in1'], motor[2]['in2'])
        direction('reverse', motor[3]['in1'], motor[3]['in2'])
    else:
        for i in motor:
            direction('stop', i['in1'], i['in2'])

time.sleep(1)
velocity('forward', speed['speed50'])
time.sleep(3)
velocity('right', speed['speed75'])
time.sleep(3)
velocity('left', speed['speed75'])
time.sleep(3)
velocity('reverse', speed['speed50'])
time.sleep(3)
velocity('stop', speed['low'])
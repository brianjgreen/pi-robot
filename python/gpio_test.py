!/bin/python
# Simple script for shutting down the raspberry Pi at the press of a button.
# Doofenshmirtz Evil Incorporated
# by Dr. Heinz Doofenshmirtz

import RPi.GPIO as GPIO
import time
import os

# Use the Broadcom SOC Pin numbers
# Setup the Pin with Internal pullups enabled and PIN in reading mode.
GPIO.setmode(GPIO.BCM)
GPIO.setup(5, GPIO.IN, pull_up_down = GPIO.PUD_UP)

"""
# Our function on what to do when the button is pressed
def Shutdown(channel):
    os.system("sudo shutdown -h now")

# Add our function to execute when the button pressed event happens
GPIO.add_event_detect(5, GPIO.FALLING, callback = Shutdown, bouncetime = 2000)
"""

# Now wait!
while 1:
    print GPIO.input(5)
    time.sleep(.05)

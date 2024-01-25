from gpiozero import LineSensor
from signal import pause

def line_detected(dev):
    print 'Line detected! {}'.format(dev)

def no_line_detected(dev):
    print 'No line detected {}'.format(dev)

sensor = LineSensor(27)
sensor.when_line = line_detected
sensor.when_no_line = no_line_detected
pause()
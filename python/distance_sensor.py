from gpiozero import DistanceSensor
from time import sleep

sensor = DistanceSensor(echo=26, trigger=21)
while True:
    print 'Distance: {}'.format(sensor.distance)
    sleep(1)
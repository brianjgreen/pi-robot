# pi-robot
Controlling a Raspberry Pi Zero W based robot with sensors

Raspberry Pi Robot 

June 2017 

 

We are building a robot platform using the low-power Raspberry Pi Zero W and the Adafruit motor shield.  Many more sensors can be added to I2C, SPI and GPIO pins. 

Raspberry Pi Zero W 

The Raspberry Pi Zero W extends the Pi Zero family. Launched at the end of February 2017, the Pi Zero W has all the functionality of the original Pi Zero but with added connectivity, consisting of: 

802.11 b/g/n wireless LAN 

Bluetooth 4.1 

Bluetooth Low Energy (BLE) 

Like the Pi Zero, it also has: 

1GHz, single-core CPU 

512MB RAM 

Mini HDMI and USB On-The-Go ports 

Micro USB power 

HAT-compatible 40-pin header 

Composite video and reset headers 

CSI camera connector 

 

GPIO 

I2C 

Use I2c connection to Adafruit motor shield. 

SDA1 (I2C data)		pin 3 

5v			pin 4 

SCL1 (I2C clock) 	pin 5 

Ground 		pin 6 

 

7 additional ground pins (one used by I2C Adafruit motor shield) 

Lots of more GPIOs with special purpose or can be used as simple GPIO pin. 

Two 3.3v DC power 

One additional 5v DC power (one used by I2C Adafruit motor shield) 

 

ShapeText BoxShapeGPIO 

Motor Shield 

Adafruit Industries Motor/Stepper/Servo Shield for Arduino v2 Kit - v2.3 

Adafruit Part 1438 

This protects the Raspberry Pi from the electrical noise, drops and surges coming from the DC motors. 

Our robot has 4 wheels.  Each wheel has a DC motor. 

The Adafruit 1438 motor shield provides: 

4 H-Bridges: TB6612 chipset provides 1.2A per bridge (3A for brief 20ms peaks) with thermal shutdown protection, internal kickback protection diodes. Can run motors on 4.5VDC to 13.5VDC. 

Up to 4 bi-directional DC motors with individual 8-bit speed selection (so, about 0.5% resolution) 

Motors automatically disabled on power-up 

Polarity protected 2-pin terminal block and jumper to connect external power, for separate logic/motor supplies 

Big terminal block connectors to easily hook up wires (18-26AWG) and power 

This board/chip uses I2C 7-bit addresses between 0x60-0x80, selectable with jumpers. 

ShapeShapeText BoxShapeText BoxShapeText BoxShapeText BoxText BoxShapeadafruit_products_1438top_LRG.jpg 

Datasheets for motor shield components 

Adafruit Part 1438 Motor Shield Schematic 

This Adafruit board is made up of an I2C PWM LED controller connected to two H-bridges. 

https://cdn-learn.adafruit.com/assets/assets/000/009/536/original/adafruit_products_mshieldv2schem.png?1396892649 

Toshiba H-bridge 

The H-bridge controls the speed and direction of the motors.  Each bridge can control two motors.  This board has two H-bridges so it can control four DC motors. 

https://cdn-shop.adafruit.com/datasheets/TB6612FNG_datasheet_en_20121101.pdf 

PCA9685 16-channel 12-bit PWM LED controller 

The I2C PWM LED controller is intended for controlling RGB LEDs.  This is instead used for controlling the H-bridges.  The I2C interface makes it easy to program from the Raspberry Pi Zero W. 

http://www.nxp.com/documents/data_sheet/PCA9685.pdf 

 

 

ShapeShapeShapeShapeShapeShapeText BoxText BoxText BoxText BoxText BoxText BoxText Box  

 

 

PWM 

IN1 

IN2 

Motor 1 

8 (0x26-0x29) 

10 (0x2e-0x31) 

9 (0x2A-0x2D) 

Motor 2 

13 (0x3a-0x3d) 

11 (0x32-0x35) 

12 (0x36-0x39) 

Motor 3 

2 (0x0e-0x11) 

4 (0x16-0x19) 

3 (0x12-0x15) 

Motor 4 

7 (0x22-0x25) 

5 (0x1a-0x1d) 

6 (0x1e-0x21) 

 

 

IN1 

IN2 

Forward 

HIGH 

LOW 

Reverse 

LOW 

HIGH 

Release 

LOW 

LOW 

Short break??? 

HIGH 

HIGH 

 

WARNING!  Always write to the LOW pin first.  (Motor will do a “short break” if both are HIGH) 

 

 

 

ON Low 

On High 

Off Low 

Off High 

HIGH (IN1, IN2) 

0x00 

0x10 

0x00 

0x00 

Speed (PWM) 

0x00 

0x00 

0x00 

0x04 (25%) 

0x00 

0x08 (50%) 

0x00 

0x0C (75%) 

Stop (PWM) 

0x00 

0x00 

0x00 

0x10 

LOW (IN1, IN2) 

0x00 

0x00 

0x00 

0x10 

 

 

Reg 0x00 = 0x10 to hold in reset 

Reg 0xFE = 0x03 for 1600 Hz frequency 

Reg 0x00 = 0xA0 to come out of reset, enable auto register increment and run! 

Shutdown Linux Button 

Need a safe way to shutdown the Raspberry Pi Linux so that the SD card does not get corrupt from sudden power off. 

Connect the reset button on the motor shield to GPIO5. 

Example Python Code 

#!/bin/python 

# Simple script for shutting down the raspberry Pi at the press of a button. 

# by Inderpreet Singh 

 

import RPi.GPIO as GPIO 

import time 

import os 

 

# Use the Broadcom SOC Pin numbers 

# Setup the Pin with Internal pullups enabled and PIN in reading mode. 

GPIO.setmode(GPIO.BCM) 

GPIO.setup(5, GPIO.IN, pull_up_down = GPIO.PUD_UP) 

 

# Our function on what to do when the button is pressed 

def Shutdown(channel): 

    os.system("sudo shutdown -h now") 

 

# Add our function to execute when the button pressed event happens 

GPIO.add_event_detect(5, GPIO.FALLING, callback = Shutdown, bouncetime = 2000) 

 

# Now wait! 

while 1: 

    time.sleep(1) 

Add to Startup 

sudo nano /etc/rc.local 

sudo python /home/pi/Scripts/shutdown_pi.py & 

 

Add LEDs to spare PWM pins 

The PWM controller has four spare channels, PWM0, PWM1, PWM14 and PWM15, that are brought to a header.  This can be used to add LEDs. 

 

Proximity sensors ultrasonic sensor HC-SR04 

Connect each sensor to two of the following GPIO pins: 6, 12, 13, 16, 19, 20. 

VCC to Pin 2 (VCC) 

GND to Pin 6 (GND) 

TRIG to Pin 12 (GPIO18) 

connect the 330Ω resistor to ECHO.  On its end you connect it to Pin 18 (GPIO24) and through a 470Ω resistor you connect it also to Pin6 (GND). 

 

 

The GPIOs in the diagram are for example only. 

 

Python Code 

 

#Libraries 

import RPi.GPIO as GPIO 

import time 

  

#GPIO Mode (BOARD / BCM) 

GPIO.setmode(GPIO.BCM) 

  

#set GPIO Pins 

GPIO_TRIGGER = 18 

GPIO_ECHO = 24 

  

#set GPIO direction (IN / OUT) 

GPIO.setup(GPIO_TRIGGER, GPIO.OUT) 

GPIO.setup(GPIO_ECHO, GPIO.IN) 

  

def distance(): 

    # set Trigger to HIGH 

    GPIO.output(GPIO_TRIGGER, True) 

  

    # set Trigger after 0.01ms to LOW 

    time.sleep(0.00001) 

    GPIO.output(GPIO_TRIGGER, False) 

  

    StartTime = time.time() 

    StopTime = time.time() 

  

    # save StartTime 

    while GPIO.input(GPIO_ECHO) == 0: 

        StartTime = time.time() 

  

    # save time of arrival 

    while GPIO.input(GPIO_ECHO) == 1: 

        StopTime = time.time() 

  

    # time difference between start and arrival 

    TimeElapsed = StopTime - StartTime 

    # multiply with the sonic speed (34300 cm/s) 

    # and divide by 2, because there and back 

    distance = (TimeElapsed * 34300) / 2 

  

    return distance 

  

if __name__ == '__main__': 

    try: 

        while True: 

            dist = distance() 

            print ("Measured Distance = %.1f cm" % dist) 

            time.sleep(1) 

  

        # Reset by pressing CTRL + C 

    except KeyboardInterrupt: 

        print("Measurement stopped by User") 

        GPIO.cleanup() 

 

 

Project Goals 

Control robot over Wifi SSH terminal using simple Python or maybe Scratch commands 

Control robot over Wifi with web page 

Control robot from phone and iPad over Bluetooth 

Robot reports sensor readings 

Temperature, humidity, etc. 

What to wear today 

Alarm (maybe) 

Add LEDs 

Add proximity sensors (one at a time) 

Program robot to avoid obstacles via the proximity sensors 

Add camera 

Program robot to follow a line using the camera 

Program robot to project camera images back to Wifi or Bluetooth connection 

Add more sensors as we wish 

Program robot to navigate itself around house or maze using sensors 

Add simple display for IP address or other short messages 

 

 

 

Technical Programming Notes 

Pseudo code from the Arundo libraries. 

Some init code 

WIRE.begin(); 

  _pwm.begin(); 

  _freq = freq; 

  _pwm.setPWMFreq(_freq);  // This is the maximum PWM frequency 

  for (uint8_t i=0; i<16; i++)  

    _pwm.setPWM(i, 0, 0); 

 

if (dcmotors[num].motornum == 0) { 

    // not init'd yet! 

    dcmotors[num].motornum = num; 

    dcmotors[num].MC = this; 

    uint8_t pwm, in1, in2; 

    if (num == 0) { 

      pwm = 8; in2 = 9; in1 = 10; 

    } else if (num == 1) { 

      pwm = 13; in2 = 12; in1 = 11; 

    } else if (num == 2) { 

      pwm = 2; in2 = 3; in1 = 4; 

    } else if (num == 3) { 

      pwm = 7; in2 = 6; in1 = 5; 

    } 

    dcmotors[num].PWMpin = pwm; 

    dcmotors[num].IN1pin = in1; 

    dcmotors[num].IN2pin = in2; 

  } 

  return &dcmotors[num]; 

 

H-bridge setting for directional movement 

 

  case FORWARD: 

    MC->setPin(IN2pin, LOW);  // take low first to avoid 'break' 

    MC->setPin(IN1pin, HIGH); 

    break; 

  case BACKWARD: 

    MC->setPin(IN1pin, LOW);  // take low first to avoid 'break' 

    MC->setPin(IN2pin, HIGH); 

    break; 

  case RELEASE: 

    MC->setPin(IN1pin, LOW); 

    MC->setPin(IN2pin, LOW); 

    break; 

 

More init code 

 

#define MOTOR1_A 2 

#define MOTOR1_B 3 

#define MOTOR2_A 1 

#define MOTOR2_B 4 

#define MOTOR4_A 0 

#define MOTOR4_B 6 

#define MOTOR3_A 5 

#define MOTOR3_B 7 

 

freq = 1600 

 

  freq *= 0.9;  // Correct for overshoot in the frequency setting (see issue #11). 

 

  float prescaleval = 25000000; 

  prescaleval /= 4096; 

  prescaleval /= freq; 

  prescaleval -= 1; 

 

Interesting init code that talks directly to the PWM chip 

 

void Adafruit_MS_PWMServoDriver::reset(void) { 

 write8(PCA9685_MODE1, 0x0); 

} 

 

uint8_t oldmode = read8(PCA9685_MODE1); 

  uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep 

  write8(PCA9685_MODE1, newmode); // go to sleep 

  write8(PCA9685_PRESCALE, prescale); // set the prescaler 

  write8(PCA9685_MODE1, oldmode); 

  delay(5); 

  write8(PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment. 

                                          // This is why the beginTransmission below was not working. 

  //  Serial.print("Mode now 0x"); Serial.println(read8(PCA9685_MODE1), HEX); 

 

void Adafruit_MS_PWMServoDriver::setPWM(uint8_t num, uint16_t on, uint16_t off) { 

  //Serial.print("Setting PWM "); Serial.print(num); Serial.print(": "); Serial.print(on); Serial.print("->"); Serial.println(off); 

 

Looks like this Arundo does not have an I2C driver.  This is bit banging. 

 

  WIRE.beginTransmission(_i2caddr); 

#if ARDUINO >= 100 

  WIRE.write(LED0_ON_L+4*num); 

  WIRE.write(on); 

  WIRE.write(on>>8); 

  WIRE.write(off); 

  WIRE.write(off>>8); 

#else 

  WIRE.send(LED0_ON_L+4*num); 

  WIRE.send((uint8_t)on); 

  WIRE.send((uint8_t)(on>>8)); 

  WIRE.send((uint8_t)off); 

  WIRE.send((uint8_t)(off>>8)); 

#endif 

  WIRE.endTransmission(); 

} 

 

uint8_t Adafruit_MS_PWMServoDriver::read8(uint8_t addr) { 

  WIRE.beginTransmission(_i2caddr); 

#if ARDUINO >= 100 

  WIRE.write(addr); 

#else 

  WIRE.send(addr); 

#endif 

  WIRE.endTransmission(); 

 

  WIRE.requestFrom((uint8_t)_i2caddr, (uint8_t)1); 

#if ARDUINO >= 100 

  return WIRE.read(); 

#else 

  return WIRE.receive(); 

#endif 

} 

 

void Adafruit_MS_PWMServoDriver::write8(uint8_t addr, uint8_t d) { 

  WIRE.beginTransmission(_i2caddr); 

#if ARDUINO >= 100 

  WIRE.write(addr); 

  WIRE.write(d); 

#else 

  WIRE.send(addr); 

  WIRE.send(d); 

#endif 

  WIRE.endTransmission(); 

} 

 

Interesting defines for the I2C PWM device 

 

#define PCA9685_SUBADR1 0x2 

#define PCA9685_SUBADR2 0x3 

#define PCA9685_SUBADR3 0x4 

 

#define PCA9685_MODE1 0x0 

#define PCA9685_PRESCALE 0xFE 

 

#define LED0_ON_L 0x6 

#define LED0_ON_H 0x7 

#define LED0_OFF_L 0x8 

#define LED0_OFF_H 0x9 

 

#define ALLLED_ON_L 0xFA 

#define ALLLED_ON_H 0xFB 

#define ALLLED_OFF_L 0xFC 

#define ALLLED_OFF_H 0xFD 

 

Demo test code for showing off the DC motors 

 

  AFMS.begin();  // create with the default frequency 1.6KHz 

  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz 

   

  // Set the speed to start, from 0 (off) to 255 (max speed) 

  myMotor->setSpeed(150); 

  myMotor->run(FORWARD); 

  // turn on motor 

  myMotor->run(RELEASE); 

 

 

void loop() { 

  uint8_t i; 

   

  Serial.print("tick"); 

 

  myMotor->run(FORWARD); 

  for (i=0; i<255; i++) { 

    myMotor->setSpeed(i);   

    delay(10); 

  } 

  for (i=255; i!=0; i--) { 

    myMotor->setSpeed(i);   

    delay(10); 

  } 

   

  Serial.print("tock"); 

 

  myMotor->run(BACKWARD); 

  for (i=0; i<255; i++) { 

    myMotor->setSpeed(i);   

    delay(10); 

  } 

  for (i=255; i!=0; i--) { 

    myMotor->setSpeed(i);   

    delay(10); 

  } 

 

  Serial.print("tech"); 

  myMotor->run(RELEASE); 

  delay(1000); 

} 

 

 

Raspberry Pi SMBus Examples 

Sample Python SMBus functions 

#!/usr/bin/python 

 

import smbus 

 

bus = smbus.SMBus(1)    # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1) 

 

DEVICE_ADDRESS = 0x15      #7 bit address (will be left shifted to add the read write bit) 

DEVICE_REG_MODE1 = 0x00 

DEVICE_REG_LEDOUT0 = 0x1d 

 

#Write a single register 

bus.write_byte_data(DEVICE_ADDRESS, DEVICE_REG_MODE1, 0x80) 

 

#Write an array of registers 

ledout_values = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff] 

bus.write_i2c_block_data(DEVICE_ADDRESS, DEVICE_REG_LEDOUT0, ledout_values) 

 

CMPS03 Compass Module Sample Python SMBus Program 

import smbus 
import time 
bus = smbus.SMBus(0) 
address = 0x60 
 
def bearing255(): 
        bear = bus.read_byte_data(address, 1) 
        return bear 
 
def bearing3599(): 
        bear1 = bus.read_byte_data(address, 2) 
        bear2 = bus.read_byte_data(address, 3) 
        bear = (bear1 << 8) + bear2 
        bear = bear/10.0 
        return bear 
 
while True: 
        bearing = bearing3599()     #this returns the value to 1 decimal place in degrees.  
        bear255 = bearing255()      #this returns the value as a byte between 0 and 255.  
        print bearing 
        print bear255 
        time.sleep(1) 

 

SRF08 Range Sensor Sample Python SMBus Program 

import smbus 
import time 
bus = smbus.SMBus(0) 
address = 0x70 
 
#SRF08 REQUIRES 5V 
 
def write(value): 
        bus.write_byte_data(address, 0, value) 
        return -1 
 
def lightlevel(): 
        light = bus.read_byte_data(address, 1) 
        return light 
 
def range(): 
        range1 = bus.read_byte_data(address, 2) 
        range2 = bus.read_byte_data(address, 3) 
        range3 = (range1 << 8) + range2 
        return range3 
 
while True: 
        write(0x51) 
        time.sleep(0.7) 
        lightlvl = lightlevel() 
        rng = range() 
        print lightlvl 
        print rng 

 

 
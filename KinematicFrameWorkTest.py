#!/usr/bin/env python 
# Author :Mr.Chanapai Chuadchum 
# Project name :RoboticArm Kinematic Sim Con 'RKSC system'
# Describetion: Use with Jetson Tk1 and Vision System function on the CUDA 
import numpy as np # Numpy function for the math and matrix calculation function 
import matplotlib.pyplot as plt  
from nanpy import (ArduinoApi,SerialManager)
from nanpy import Servo
from nanpy import serial_manager 
from nanpy import CapacitiveSensor # Capacitive sensor input 
from nanpy import DHT,DallasTemperature 
from nanpy.arduinotree import ArduinoTree # Arduino Tree function 
import time # Time control delaytion
#Connection of the System on the serial communication system 
connection = SerialManager('/dev/ttyACM0',115200)  # Serial communication via MCU function 
a = ArduinoApi(connection=connection) # Connection with the serial magnager 
connectionBody = SerialManager('/dev/ttyACM1',115200) # Serial sensing and control 
b = ArduinoApi(connection=connectionBody) # The Body and sensing connection for the system of therobotic arm 
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
          # Servo part for the robotic arm 
servo3e = Servo(3)  
servo3u = Servo(4)
servo1u = Servo(5) 
servo1e = Servo(8)
servo2u = Servo(9) 
servo2e = Servo(10)  
          # Servo for the robotic wrist of the arm 
servoWrist= Servo(43) # Wrist servo  
servowristrotate = Servo(44) # Wrist rotate servo 
 # Stepper motor at the finger
#Finger 1 
a.pinMode(11,a.OUTPUT) #dir
a.pinMode(12,a.OUTPUT) #step 
#Finger 2  
a.pinMode(2,a.OUTPUT)  #dir
a.pinMode(6,a.OUTPUT)  #step 
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
   # Stepper motor at the Body arm 
#Elbow 
a.pinMode(23,a.OUTPUT) #dir
a.pinMode(24,a.OUTPUT) #step
#Shoulder  
a.pinMode(25,a.OUTPUT) #dir
a.pinMode(26,a.OUTPUT) #step
#Base 
a.pinMode(27,a.OUTPUT) #dir
a.pinMode(28,a.OUTPUT) #step
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    #Stepper motor Motor power cotrol function
a.pinMode(29,a.OUTPUT) #Stepper motor Base power on/off
a.pinMode(30,a.OUTOUT) #Stepper motor Shoulder power on/off 
a.pinMode(31,a.OUTPUT) #Stepper motor Elbow power on/off 
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
a.pinMode(32,a.INPUT)
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 # Function of the i2c Address communication
def ItwoCmain(): 
        connection = SerialManager(sleep_after_connect=2)
        connection.open() 
        print(connection.device)
        a = ArduinoTree(connection=connection)
        master = I2C_Master(a.wire)
        master.send(I2C_ADDRESS,[0b00001000])

def BaseJoint(dir,step,speed,timing):  # Robotic Base Joint control  
    if dir == 1 : 
         a.digitalWrite(27,a.HIGH) 
         for move in range(0,step,1): 
              a.digitalWrite(28,a.HIGH) 
              time.sleep(speed)
              a.digitalWrite(28,a.LOW)  
    if dir == 0:
         a.digitalWrite(27,a.LOW) 
         for move in range(0,step,1): 
            a.digitalWrite(28,a.HIGH)
            time.sleep(speed)
            a.digitalWrite(28,a.LOW) 
            time.sleep(speed) 
def ElbowJoint(dir,step,speed,timing): # Robotic Elbow Joint control 
    if dir == 1: 
        a.digitalWrite(23,a.HIGH) #Forward
        for move in range(0,step,1):
            a.digitalWrite(24,a.HIGH)
            time.sleep(speed)
            a.ditialWrite(24,a.LOW)
            time.sleep(speed)
    if dir == 0: 
        a.digitalWrite(23,a.LOW)  # Inverse
        for move in range(0,step,1):
            a.digitalWrite(24,a.HIGH)
            time.sleep(speed)
            a.digitalWrite(24,a.LOW)
            time.sleep(speed)        
def ShouelderJoint(dir,step,speed,timing): # Robotic Shoulder Joint control
    if dir == 1: 
        a.digitalWrite(25,a.HIGH) #Forward
        for move in range(0,step,1):
            a.digitalWrite(26,a.HIGH)
            time.sleep(speed)
            a.ditialWrite(26,a.LOW)
            time.sleep(speed)
    if dir == 0: 
        a.digitalWrite(25,a.LOW)  # Inverse
        for move in range(0,step,1):
            a.digitalWrite(26,a.HIGH)
            time.sleep(speed)
            a.digitalWrite(26,a.LOW)
            time.sleep(speed)        
def Wrist_servo(dir,AngleF,AngleI,speed):
   if dir == 1: 
    for move in range(AngleF,AngleI,1):
        servoWrist.write(move)
        time.sleep(speed)
   if dir == 0:
    for move in range(AngleI,AngleF,-1):
        servoWrist.write(move)
        time.sleep(speed)     
def Wristrotate_servo(dir,AngleF,AngleI,speed): 
   if dir == 1:  
     for move in range(AngleF,AngleI,speed): 
        servoWrist.write(move)
        time.sleep(speed) 
   if dir == 0: 
     for move in range(AngleI,AngleF,speed): 
         servoWrist.write(move)
         time.sleep(speed) 
def StepperFinger2(dir,step,speed,timing): # Robotic Finger joint control 
    if dir == 1: 
        a.digitalWrite(23,a.HIGH) #Forward
        for move in range(0,step,1):
            a.digitalWrite(24,a.HIGH)
            time.sleep(speed)
            a.ditialWrite(24,a.LOW)
            time.sleep(speed)
    if dir == 0: 
        a.digitalWrite(23,a.LOW)  # Inverse
        for move in range(0,step,1):
            a.digitalWrite(24,a.HIGH)
            time.sleep(speed)
            a.digitalWrite(24,a.LOW)
            time.sleep(speed)        
def StepperFinger3(dir,step,speed,timing):
       if dir == 1: 
         a.digitalWrite(23,a.HIGH) #Forward
         for move in range(0,step,1):
            a.digitalWrite(24,a.HIGH)
            time.sleep(speed)
            a.ditialWrite(24,a.LOW)
            time.sleep(speed)
       if dir == 0: 
         a.digitalWrite(23,a.LOW)  # Inverse
         for move in range(0,step,1):
            a.digitalWrite(24,a.HIGH)
            time.sleep(speed)
            a.digitalWrite(24,a.LOW)
            time.sleep(speed)     
def  Hand_SensoryFunction(x,y,z,State):
    if State == 1 : #Catch 
       
    if State == 0 : #Catch 
       
def BarrettFingercontrol(dir1,dir2,dir3,dir4,dir5,AngleI1,AngleI2,AngleI3,AngleF1,AnngleF2,AngleF3,Speed,step2,step3,timing):
      #Finger 1 functional positioning control 
    Finger1(dir1,AngleI1,AngleF1,Speed)
      #Finger 2 functional position control 
    Finger2(dir2,AngleI2,AngleF2,Speed)
      #Finger 3 funtional position control 
    Finger3(dir3,AngleI3,AngleF3,Speed)
      #Finger 3 rotator 
    StepperFinger3(dir5,step3,speed,timing)  
      #Finger 2 rotator 
    StepperFinger2(dir4,step2,speed,timing)
  # Finger control for the robot barrett hand 
def Finger1(dir,AngleI,AngleF,speed):
    if dir == 1: 
        for move in range(AngleF,AngleI,1): #Finger move catch 
            servo1u.write(4+move) 
            servo1e.write(5+move)  #Finger 1 move catch  
            time.sleep(speed) # Speed control function for the servo motor
    else if dir == 0 :
        for move in  range(AngleI,AngleF,-1):
            servo1u.write(4+move) 
            servo1e.write(5+move)
            time.sleep(speed) #Speed control function for the servo motor 
def Finger2(dir,AngleI,AngleF,speed):
     if dir == 1: 
         for move in range(AngleF,AngleI,1): #Finger move catch 
            servo2u.write(170-move) 
            servo2e.write(move)
            time.sleep(speed) # Speed control function for the servo motor 
     else if dir == 0 :
         for move in range(AngleI,AngleF,-1):
             servo2u.write(180-move)
             servo2e.write(move)
             time.sleep(speed)  #Speed control function for the servo motor 
def Finger3(dir,AngleI,AngleF,speed): 
      if dir == 1: 
         for move in range(AngleF,AngleI,1): 
             servo3e.write(10-move)
             servo3u.write(move)
             time.sleep(speed) # speed control function for the servo motor 
      else if dir == 0:
         for move in range(AngleI,AngleF,-1):
             servo3e.write(move)
             servo3u.write(move)
             time.sleep(speed) #Speed control function for the  servo motor 
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

if __name__ = '__manin__':      # Working and End not the loop 
    #ItwoCmain() 

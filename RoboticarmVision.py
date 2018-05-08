#!/usr/bin/env python   
#Author code : Mr.Chanapai Chuadchum 
#Project name: Robotic arm and Vision system    
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
import sys # Data base function for the robotic arm using this to keep the data of the machine learning function 
import math  # math function for the complex physical body calculation
import numpy as np  #numpy for math function vision and sound  
import cv2 #Robotic Arm Vision System for  
from nanpy import (ArduinoApi,SerialManager) #Robotic Arm Serial firmwear function 
from nanpy import servo 
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 # The matplot 3d visual graph fuction 
import matplotlib.pyplot as plt  # matplot lib for the graph report 
from mpl_toolkits.mplot3d import Axes3D # plot 3D graph dynamic analysis 
 #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
import csv  # Writing the csv back to the machine learning algorithm  
from colletions import deque  #Collection data
from sklearn.feature_extraction.text import TfidfVectorizer #Mchine learning part 
from sklearn.linear_model.logistic import LogisticRegression # Machine learning logistic part  
from sklearn.cross_validation import train_test_split #Data splitter test 
from time import sleep # Delay function for the timer 
import serial #Serial communication for reading the message function 
import microgear.client as microgear #Sending the message from the robotic arm  
import logging  # Showing th logging data function 
#Serial function to recieve the sensor value 
ser = serial.Serial("/dev/ttyUSB0",115200) #Serial fixed baudrate  
#Camera activation part for the opencv object recognition realtime train  
cap = cv2.VideoCapture(0)  # camera 1
#cap2 = cv2.VideoCapture(1) # camera 2
connection = SerialManager()  # Serial manager function for the MCU controller function 
a =  AdrduinoApi(connection=connection) #The connection function for the Arduino Api 
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
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
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
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
         #Sensor record function for training 
def SensorsRecord():
    with open('sensorsmap.csv','wb') as csvfile: 
     spamwriter = csv.writer(csvfile,delimiter='',
                                quotechar=',',quoting=csv.QUOTE_MINIMAL)

     spamwriter.writerow(ser.readline()) # Write the sensor record for the sensory and joint angle feed back 
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
               #Stepper motors parts for the high precision joint of the robotic arm 
def BaseJoint(dir,step,speed,timing): 
       if dir == 1: 
          a.digitalWrite(27,a.HIGH) # Forward 
          for move in range(0,step,1): 
               a.digitalWrite(28,a.HIGH)
               time.sleep(speed)
               a.digitalWrite(28,a.LOW)
               time.sleep(speed) 
       time.sleep(timing) # timing for the stepper  
       if dir == 0:
          a.digitalWrite(27,a.LOW) # Backward 
          for move in range(0,step,1):
              a.digitalWrite(28,a.HIGH)
              time.sleep(speed)
              a.digitalWrite(28,a.LOW)
              time.sleep(speed) 
       time.sleep(timing) # timing for the stepper    
def ShoulderJoint(dir,step,speed,timing): 
       if dir == 1: 
          a.digitalWrite(25,a.HIGH) # Forward 
          for move in range(0,step,1): 
               a.digitalWrite(26,a.HIGH)
               time.sleep(speed)
               a.digitalWrite(26,a.LOW)
               time.sleep(speed) 
       time.sleep(timing) # timing for the stepper  
       if dir == 0:
          a.digitalWrite(25,a.LOW) #Backward 
          for move in range(0,step,1):
              a.digitalWrite(26,a.HIGH)
              time.sleep(speed)
              a.digitalWrite(26,a.LOW)
              time.sleep(speed) 
       time.sleep(timing) # timing for the stepper  
def ElbowJoint(dir,step,speed,timing): 
       if dir == 1: 
          a.digitalWrite(23,a.HIGH) # Forward 
          for move in range(0,step,1): 
               a.digitalWrite(24,a.HIGH)
               time.sleep(speed)
               a.digitalWrite(24,a.LOW)
               time.sleep(speed) 
       time.sleep(timing) # timing for the stepper  
       if dir == 0:
          a.digitalWrite(23,a.LOW) # Backward 
          for move in range(0,step,1):
              a.digitalWrite(24,a.HIGH)
              time.sleep(speed)
              a.digitalWrite(24,a.LOW)
              time.sleep(speed) 
       time.sleep(timing) # timing for the stepper
       #Finger stepper motor for the Barrett hand type  
def StepperFinger3(dir,step,speed,timing): 
       if dir == 1: 
          a.digitalWrite(11,a.HIGH) # Forward 
          for move in range(0,step,1): 
               a.digitalWrite(12,a.HIGH)
               time.sleep(speed)
               a.digitalWrite(12,a.LOW)
               time.sleep(speed) 
       time.sleep(timing) # timing for the stepper  
       if dir == 0:
          a.digitalWrite(11,a.LOW) # Backward 
          for move in range(0,step,1):
              a.digitalWrite(12,a.HIGH)
              time.sleep(speed)
              a.digitalWrite(12,a.LOW)
              time.sleep(speed) 
       time.sleep(timing) # timing for the stepper 
def StepperFinger2(dir,step,speed,timing): 
       if dir == 1: 
          a.digitalWrite(2,a.HIGH) # Forward 
          for move in range(0,step,1): 
               a.digitalWrite(6,a.HIGH)
               time.sleep(speed)
               a.digitalWrite(6,a.LOW)
               time.sleep(speed)  
       time.sleep(timing) # timing for the stepper  
       if dir == 0:
          a.digitalWrite(2,a.LOW) # Backward 
          for move in range(0,step,1):
              a.digitalWrite(6,a.HIGH)
              time.sleep(speed)
              a.digitalWrite(6,a.LOW)
              time.sleep(speed) 
       time.sleep(timing) # timing for the stepper 
    # Barrett hand function for the catching option on the finger 
def Kinematicframework_Algorithm(BeaconRSSI,x,y,z,AngleW,speed,timing,GearratioS,GearratioB): # Robotic positioning and and location track 
  #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
     #Physical details 
     l1 = 34   # Lenght for the shoulder 
     l2 = 34   # Lenght for the Elbow 
     l3 = 15   # Lenght for the wrist 
     lfinger = 23.5 # Lenght for the wrist to the fingers      
  #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    #Angle to distance calculation  (Feed Back ) ! sensor  
   # Y lenght   Arm
       #Distance Calculation from the angle 
    # Disgram 
    # Feed back Angle ====> distance 
    # compare distance with the position X,Y,Z calculation in the 3D space 
    # Cosine distance calculation 
    dy = math.sqrt(math.pow(l1,2) + math.pow(l2,2) - 2*l1*l2*math.cos(math.raidians(AngleE)) # distance output of the arm 
    
    AngleO = math.atan(z/dy) # The Angle base calculation  
     #Oriented angl for the whole arm 
    AngleOriented = math.degrees(math.atan(y/x))
    if AngleOriented > 0:
      Step = AngleOriented/math.radians(GearratioB/(200*math.pi)) 
      for angle in range(0,Step,1):
        BaseJoint(1,200,speed,timing)
    if AnglOriented < 0:
    Step = AngleOriented/math.radians(GearratioB/(200*math.pi))
      for angle in range(StepI,Step,1):
        BaseJoint(0,200,speed,timing)
    
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>               
    #Detail for the lenght of the robot and angle turn on the gear 
def OffsetbodyDat(AnglS,AngleE,AngleB):
    ls = 34   # Lenght Shoulder 
    le = 34   # Lenght Elbow 
    lw = 15   # Lenght Wrist 
#case of the lenght equality function 
   if AngleShoulder == AngleS:
      dS = ls*math.cos(AngleS)   #  
     # return the ds back for the ground distance back
     if AngleElbow == AngleS: 
        dE = ls*math.cos(AngleE)
        return dS+dE+dW      # return the lenght
    #Barrett hand control function 
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
    # Wrist part calculation 
def Wrist_servo(dir,AngleF,AngleI,speed):
  if dir == 1: 
    for move in range(AngleF,AngleI,1):
        servoWrist.write(move)
        time.sleep(speed)
  if dir == 0:
    for move in range(AngleI,AngleF,-1):
        servoWrist.write(move)
        time.sleep(speed)     
def Wristrotate_servo(AngleF,AngleI,speed): 
    for move in range(AngleF,AngleI,speed): 
        servoWrist.write(move)
        time.sleep(speed)  
# Matplot function for the 3d visuallize 
def ThreeDVersualGraph(x,y,z): 
    fig = plt.figure() #Fig function for the system of the 3D space option  
    ax = Axes3D(fig) 
    dx = np.arange()
    dy = np.arange()
    dx,dy = np.meshgrid(dx,dy) 
    R = np.sqrt(dx**2 + dy**2)
    Z = np.sin(R)
    ax.plot_surface(dx,dy,Z,rstride=1,cstride=1,cmap='cool') #Showng the graph with the cool color 
    plt.show() #Showing the function of the function of the graph  
while(True):   # While loop operating the camera and functional programming control
    # Capture frame-by-frame
    ret, frame = cap.read()
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


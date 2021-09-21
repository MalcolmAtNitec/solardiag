#!/usr/bin/env python
### BEGIN INIT INFO
# Provides:          sample.py
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Start daemon at boot time
# Description:       Enable service provided by daemon.
### END INIT INFO
#####
#
#   Solar tracker v1
import os
import time
import sys
import paho.mqtt.client as mqtt
import paho.mqtt.subscribe as subscribe
import json
import socket
import datetime
import subprocess
import threading
import RPi.GPIO as GPIO   # Import the GPIO library.
import time               # Import time library
import SolarConstants
import requests
import smtplib
import smbus
import serial




TimeZone = -5
AParkPosition = 0
ZParkPosition = 0
# Ottawa
#Longitude = -75.695
#Latitude = 45.4125 
# ?? Linwood
Longitude = -81.2453
Latitude = 42.9849 
MotorEastTick = 52
MotorWestTick = -280
ParkOnOff = 0
TrackerOnOff = 1
ZenithOffset = 0
AzimuthOffset = 0
AzimuthAngle = 0
ZenithAngle = 0
### Push buttons are off
AzimuthPSL = 0
AzimuthNSL = 0
ZenithPSL = 0
ZenithNSL = 0
AzimuthPSLValue = 0
AzimuthNSLValue = 0
ZenithPSLValue = 0
ZenithNSLValue = 0
ReverseEngine  = 0
ZReverseEngine  = 0
pwmMotor1 = 0
pwmMotor2 = 0
Motor1NotMoving = 0
Motor2NotMoving = 0
PreviousMotor1Pos = 0
PreviousMotor2Pos = 0
MovingMotor1 = 0
MovingMotor2 = 0
NightAngle = 0
NightPosition = 0
Motor1WaitFlag = 0
MoveToNightPosition = 0
MotorState = 0
MotorState2 = 0
TrackerEnable = 0
ParkWindSpeed = 0
NightPosition = 0
wind = 0
temp = 0


#################################################
# GPIO Pins are based on board pins
#
# Motor 1 Azimuth based on pin number on connector
Motor1Dir    =  37
Motor1HallA  =  8
Motor1HallB  =  10
Motor1PWM    =  31
LimitSwitch1 =  12
LimitSwitch1n = 16

# Motor 2 Zenith  based on pin number on connector
Motor2Dir    =  18
Motor2HallA  =  22
Motor2HallB  =  40
Motor2PWM    =  28 
LimitSwitch2 =  36
LimitSwitch2n = 38

DigOut0       = 36  # new
DigOut1       = 35
DigOut2       = 12  # new 58
DigOut3       = 13  # new 34

DigIn0        = 18 # new 55
DigIn1        = 22 # new 51
DigIn2        = 37 # new 41
DigIn3        = 23 # new 25
DigIn4        = 7 # new 46
DigIn5        = 11 # new 47

#ADC MUX pins
MUXSel0       = 39
MUXSel1       = 40
MUXSel2       = 44
CalEn         = 38

#  MCP4725 defaults to address 0x60
ADCAddress = 0xA8
GPSAddress = 0x3A
CELLAddress = 0xA8

motor1Pos = 0 # steps that motor 1 has moved.
Motor1levA = 0
Motor1levB = 0
StepsToStop = 0
NewPosition = 0
Motor1State = 0
Motor1Pos = 0
Motor1Direction = 0
Motor1Positive = 0
Motor1Negative = 0
Motor1InMotion = False
Limit1PosReached = 0
Limit1NegReached = 0


Motor2levA = 0
Motor2levB = 0
Motor2Pos = 0
Motor2Direction = 0
Motor2Positive = 0
Motor2Negative = 0
Motor2InMotion = False
Limit2PosReached = 0
Limit2NegReached = 0
Motor2State = 0
StepsToStop2 = 0

"""
Set the direction and speed of motor 1
"""
def SetMotorDirection1(MotorDirection):
    print("Set motor 1 direction ", MotorDirection)
    setDutyMotor1(0) 
    #time.sleep(5)    # allow the motor to stop
    if MotorDirection == SolarConstants.MOTOR_FORWARD:  # forward
        print("forward")
        GPIO.output(Motor1Dir,GPIO.LOW)
        #time.sleep(2)
        setDutyMotor1(100) 
    elif MotorDirection == SolarConstants.MOTOR_REVERSE:
        print("reverse")
        GPIO.output(Motor1Dir,GPIO.HIGH)
        #time.sleep(2)
        setDutyMotor1(100) 
    else: # stop
        print("stopped")
        setDutyMotor1(0) 

   
"""
Set the direction and speed of motor 2
"""
def SetMotorDirection2(MotorDirection):
    print("Set motor 2 direction ", MotorDirection)
    setDutyMotor2(0) 
    time.sleep(5)    # allow the motor to stop
    if MotorDirection == SolarConstants.MOTOR_FORWARD:  # forward
        print("motor 2 forward")
        GPIO.output(Motor2Dir,GPIO.LOW)
        time.sleep(2)
        setDutyMotor2(100) 
    elif MotorDirection == SolarConstants.MOTOR_REVERSE:
        print("motor 2 reverse")
        GPIO.output(Motor2Dir,GPIO.HIGH)
        time.sleep(2)
        setDutyMotor2(100) 
    else: # stop
        print("stopped")
        setDutyMotor2(0) 

"""
Set the direction, distance and speed of the motor
"""
def MoveMotor1StepsDirection(MotorDirection, NumSteps, DutyCycle):
    global StepsToStop
    global Motor1Pos
    global MovingMotor1
    global ReverseEngine

    print("Set motor 1 steps and direction ", NumSteps, MotorDirection, Motor1Pos)
    setDutyMotor1(0) 
    time.sleep(5)    # allow the motor to stop
    StepsToStop = NumSteps
    if MotorDirection == SolarConstants.MOTOR_FORWARD:  # forward
        print("forward", NumSteps, Motor1Pos)
        MovingMotor1 = 1
        if ReverseEngine == 0:
            GPIO.output(Motor1Dir,GPIO.LOW)
        else:
            GPIO.output(Motor1Dir,GPIO.HIGH)
        time.sleep(0.2)
        setDutyMotor1(DutyCycle) 
    elif MotorDirection == SolarConstants.MOTOR_REVERSE:
        print("reverse")
        MovingMotor1 = 1
        if ReverseEngine == 0:
            GPIO.output(Motor1Dir,GPIO.HIGH)
        else:
            GPIO.output(Motor1Dir,GPIO.LOW)
        time.sleep(0.2)
        setDutyMotor1(DutyCycle) 
    else: # stop
        print("stopped")
        setDutyMotor1(0) 

"""
Set the direction, distance and speed of the motor
"""
def MoveMotor2StepsDirection(MotorDirection, NumSteps, DutyCycle):
    global StepsToStop2
    global Motor2Pos
    global MovingMotor2
    global ZReverseEngine

    print("Set motor 2 steps and direction ", NumSteps, MotorDirection, Motor2Pos)
    setDutyMotor2(0) 
    time.sleep(5)    # allow the motor to stop
    StepsToStop2 = NumSteps
    if MotorDirection == SolarConstants.MOTOR_FORWARD:  # forward
        print("forward", NumSteps, Motor2Pos)
        MovingMotor2 = 1
        if ZReverseEngine == 0:
            GPIO.output(Motor2Dir,GPIO.LOW)
        else:
            GPIO.output(Motor2Dir,GPIO.HIGH)
        time.sleep(0.2)
        setDutyMotor2(DutyCycle) 
    elif MotorDirection == SolarConstants.MOTOR_REVERSE:
        print("reverse")
        MovingMotor2 = 1
        if ZReverseEngine == 0:
            GPIO.output(Motor2Dir,GPIO.HIGH)
        else:
            GPIO.output(Motor2Dir,GPIO.LOW)
        time.sleep(0.2)
        setDutyMotor2(DutyCycle) 
    else: # stop
        print("stopped")
        setDutyMotor2(0) 

def PWM_Motor_Control():
        time.sleep(0.05)
        setDutyMotor1(65) #65)

def weather_thread(name):
    while True:
        time.sleep(180)
        GetWeatherData()


def  SetPanelToNight():
     global NightPosition
     global MotorState

     if MotorState != 1:
         print("Setting panel to night position: ", NightPosition)
         MotorState = 1
         if NightPosition == 1:
             MoveToPosition(AParkPosition)
         else:
             MoveToPosition(AzimuthOffset)

def  SetPanelToNight2():
     global NightPosition
     global MotorState2

     if MotorState2 != 1:
         MotorState2 = 1
         print("Setting motor 2 to night position: ", NightPosition)
         if NightPosition == 1:
             MoveToPosition2(ZParkPosition)
         else:
             MoveToPosition2(ZenithOffset)

##############################################################################################
#
# motor 1 thread is used to control the direction of the sun.
# This is the azimuth angle the suns direction from North
#
def motor1_thread(name):
    global start_time
    global Motor1Pos
    global Motor1State
    global Motor1InMotion
    global Motor1WaitFlag
    global MovingMotor1 
    global AzimuthAngle
    global MoveToNightPosition

    CurrentAzimuthAngle = 0
    # First find how after a power on
    MovingMotor1 = 1
    #setDutyMotor2(100)   # mdm debug
    FindLimitSwitches1()
    MovingMotor1 = 0
    print("Limits Motor 1: ", Motor1Negative, Motor1Positive)
    SetMotorDirection1(SolarConstants.MOTOR_STOP)
# mdm MoveToPosition(MotorEastTick)
# mdm time.sleep(20)
# mdm MoveToPosition(270)
# mdm time.sleep(20)
# mdm MoveToPosition(50)
# mdm time.sleep(20)
# mdm later    MoveToPosition(AzimuthOffset)
#    time.sleep(10)
    PreviousMotorPosition = Motor1Pos
    start_time = datetime.datetime.now() 
    # First check if the panel is at the limit switch
    Limit_A = GPIO.input(LimitSwitch1)
    Limit_B = GPIO.input(LimitSwitch1n)
    print("Current limit switch readings: ", Limit_A, Limit_B)
#    if Limit_A == 1:
#        NewPosition = 25
#        print("Startup Moving away from limit switch 1")
#        MoveMotor1StepsDirection(SolarConstants.MOTOR_FORWARD,NewPosition, 100) 
#        time.sleep(1) # wait for move to complete
#        print("Now at: ", Motor1Pos)
#    if Limit_B == 1:
#        NewPosition = -25
#        print("Startup Moving away from limit switch 1")
##        MoveMotor1StepsDirection(SolarConstants.MOTOR_REVERSE,NewPosition, 100)
#        time.sleep(1) # wait for move to complete
#        print("Now at: ", Motor1Pos)
    Motor1State = 0
    SetMotorDirection1(SolarConstants.MOTOR_STOP)
    print("Motor 1 complete")
    Motor1WaitFlag = 0
    while True:
        time.sleep(1.0)
        Limit_A = GPIO.input(LimitSwitch1)
        Limit_B = GPIO.input(LimitSwitch1n)
        if Motor1State == 1:
           MoveMotor1StepsDirection(SolarConstants.MOTOR_FORWARD,NewPosition, 100) 
           Motor1State = 2
    #    if Motor1State == 2 and Limit_A == 1:
    #        NewPosition = 5
    #        print("Moving away from limit switch 1")
    #        MoveMotor1StepsDirection(SolarConstants.MOTOR_FORWARD,NewPosition, 100) 
    #        time.sleep(1) # wait for move to complete
    #        print("Now at: ", Motor1Pos)
    #    if Motor1State == 2 and Limit_B == 1:
    #        NewPosition = -2;
    #        print("Moving away from limit switch 1")
    #        MoveMotor1StepsDirection(SolarConstants.MOTOR_REVERSE,NewPosition, 100) 
    #        time.sleep(1) # wait for move to complete
    #        print("Now at: ", Motor1Pos)
        if TrackerOnOff == 1:
            if MoveToNightPosition == 1:
                SetPanelToNight()
            elif CurrentAzimuthAngle != AzimuthAngle: 
                print("Moving tracker Azimuth")
                CurrentAzimuthAngle = AzimuthAngle
                NewPosition = int(CurrentAzimuthAngle * 1.311)
                MoveToPosition(CurrentAzimuthAngle)
                #print("New Position: ", NewPosition, " ", Motor1Pos, " state", Motor1State, " positive:", Motor1Positive, " negative:", Motor1Negative)
                #if (Motor1State == 0 and NewPosition > Motor1Pos ):
                #   Motor1State = 1
        if ParkOnOff == 1:
            MoveToPosition(AParkPosition)
            time.sleep(10)
#        print("motor", PreviousMotorPosition, Motor1Pos)
        
##############################################################################################
#
# motor 2 thread is used to control the panel above the horizon.
#
# This is the zenith angle the suns direction above he horizon 
#
def motor2_thread(name):
    global start_time
    global Motor2Pos
    global Motor2State
    global MovingMotor2
    global ZenithAngle
    global MoveToNightPosition


    CurrentZenithAngle = 0
# First find how after a power on
    MovingMotor2 = 1
    FindLimitSwitches2()
    MovingMotor2 = 0
    print("Limits Motor 2: ", Motor2Negative, Motor2Positive)
# mdm MoveToPosition2(MotorEastTick)
# mdm time.sleep(20)
# mdm MoveToPosition2(270)
# mdm time.sleep(20)
# mdm MoveToPosition2(50)
# mdm time.sleep(20)
#mdm add later    MoveToPosition2(ZenithOffset)
    time.sleep(10)
    PreviousMotorPosition = Motor2Pos
    start_time = datetime.datetime.now() 
    # First check if the panel is at the limit switch
    Limit_A = GPIO.input(LimitSwitch2)
    Limit_B = GPIO.input(LimitSwitch2n)
    print("Current limit switch readings: ", Limit_A, Limit_B)
    #if Limit_A == 1:
    #    NewPosition2 = 25
    #    print("Startup Moving away from limit switch 1")
    #    MoveMotor2StepsDirection(SolarConstants.MOTOR_FORWARD,NewPosition2, 100) 
    #    time.sleep(1) # wait for move to complete
    #    print("Now at: ", Motor1Pos)
    #if Limit_B == 1:
    #    NewPosition2 = -25
    #    print("Startup Moving away from limit switch 1")
    #    MoveMotor2StepsDirection(SolarConstants.MOTOR_REVERSE,NewPosition2, 100)
    #    time.sleep(1) # wait for move to complete
    Motor2State = 0
    SetMotorDirection2(SolarConstants.MOTOR_STOP)
    print("Motor Thread 2 running")
    while True:
        time.sleep(1.0)
        Limit_A = GPIO.input(LimitSwitch2)
        Limit_B = GPIO.input(LimitSwitch2n)
        if Motor2State == 1:
           MoveMotor2StepsDirection(SolarConstants.MOTOR_FORWARD,NewPosition2, 20) 
           Motor2State = 2
    #    if Motor2State == 2 and Limit_A == 1:
    #        NewPosition2 = 5
    #        print("Moving away from limit switch 1")
    #        MoveMotor2StepsDirection(SolarConstants.MOTOR_FORWARD,NewPosition2, 100) 
    #        time.sleep(1) # wait for move to complete
    #        print("Now at: ", Motor2Pos)
    #    if Motor2State == 2 and Limit_B == 1:
    #        NewPosition = -2;
    #        print("Moving away from limit switch 1")
    #        MoveMotor2StepsDirection(SolarConstants.MOTOR_REVERSE,NewPosition2, 100) 
    #        time.sleep(1) # wait for move to complete
    #        print("Now at: ", Motor2Pos)
        if TrackerOnOff == 1:
            if MoveToNightPosition == 1:
                SetPanelToNight2()
            elif CurrentZenithAngle != ZenithAngle: 
                print("Moving tracker Zenith")
                CurrentZenithAngle = ZenithAngle
                NewPosition = int(CurrentZenithAngle * 1.311)
                MoveToPosition2(CurrentZenithAngle)
                #print("New Position: ", NewPosition, " ", Motor2Pos, " state", Motor2State, " positive:", Motor2Positive, " negative:", Motor2Negative)
                #if (Motor1State == 0 and NewPosition2 > Motor2Pos ):
                #   Motor1State = 1
        if ParkOnOff == 1:
            MoveToPosition2(ZParkPosition)
            time.sleep(10)


##############################
# This will trigger when either of the motor 1 limits are reached 
def LimitSwitchPulse1(gpio):
      global Limit1PosReached 
      global Limit1NegReached 
      global LimitSwitch1
      global LimitSwitch1n

      Motor1levA = GPIO.input(LimitSwitch1) 
      Motor1levB = GPIO.input(LimitSwitch1n) 
      """
      """
      if Motor1levA == 0 and Motor1levB == 0:
          return
      print("Limit reached", gpio, Motor1levA, Motor1levB)
      if gpio == LimitSwitch1:
          Limit1PosReached = 1;
      if gpio == LimitSwitch1n:
          Limit1NegReached = 1;
      setDutyMotor1(0) 

##############################
# This will trigger when either of the motor 1 limits are reached 
def LimitSwitchPulse2(gpio):
      global Limit2PosReached 
      global Limit2NegReached 
      global LimitSwitch2
      global LimitSwitch2n

      Motor2levA = GPIO.input(LimitSwitch2) 
      Motor2levB = GPIO.input(LimitSwitch2n) 
      """
      """
      if Motor2levA == 0 and Motor2levB == 0:
          return
      print("Limit reached", gpio, Motor2levA, Motor2levB)
      if gpio == LimitSwitch2:
          Limit2PosReached = 1;
      if gpio == LimitSwitch2n:
          Limit2NegReached = 1;
      setDutyMotor2(0) 

##############################
# determine step and direction
def Motor1Pulse(gpio):
      global Motor1levA
      global Motor1levB
      global Motor1Pos
      global MovingMotor2

      """
      Decode the rotary encoder pulse.

                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1

             +---------+         +---------+            0
             |         |         |         |
         B   |         |         |         |
             |         |         |         |
         ----+         +---------+         +---------+  1
      """

#      Motor1levA = 0
#      Motor1levB = 0
    #  if MovingMotor2 == 1:
    #      return
      if gpio == Motor1HallA or gpio == Motor1HallB:
          if gpio == Motor1HallA:
              Motor1levA = GPIO.input(Motor1HallA) 
          else:
              Motor1levB =  GPIO.input(Motor1HallB)
#      print(gpio, Motor1levA,Motor1levB)
          if gpio == Motor1HallA and Motor1levA == 1:
              if Motor1levB == 1:
                UpdateMotor1Steps(1) 
          elif gpio == Motor1HallB and Motor1levB == 1:
              if Motor1levA == 1:
                UpdateMotor1Steps(-1)

def UpdateMotor1Steps(way):
    global Motor1Pos
    global Motor1State
    global StepsToStop
    global Motor1Direction
    global MovingMotor1

    Motor1Pos += way
    if way == 1:
        Motor1Direction = 1
    else:
        Motor1Direction = 0

    print("1 - ", Motor1Pos, StepsToStop)
    if StepsToStop == Motor1Pos: # and Motor1State == 2:
      setDutyMotor1(0) 
      MovingMotor1 = 0
      Motor1State = 0
    if Motor1Pos >= 472 or Motor1Pos <= -472:
      print("Limit 1 detected ", Motor1Pos)
      MovingMotor1 = 0
      setDutyMotor1(0) 
      Motor1State = 0

##############################
# determine step and direction
def Motor2Pulse(gpio):
      global Motor2levA
      global Motor2levB
      global Motor2Pos
      global MovingMotor1

      """
      Decode the rotary encoder pulse.

                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1

             +---------+         +---------+            0
             |         |         |         |
         B   |         |         |         |
             |         |         |         |
         ----+         +---------+         +---------+  1
      """

    #  Motor2levA = 0
    #  Motor2levB = 0
    #  if MovingMotor1 == 1:
    #      return
      if gpio == Motor2HallA or gpio == Motor2HallB:
          if gpio == Motor2HallA:
              Motor2levA = GPIO.input(Motor2HallA) 
          else:
              Motor2levB =  GPIO.input(Motor2HallB)
 #     print(gpio, Motor2levA,Motor2levB)
          if gpio == Motor2HallA and Motor2levA == 1:
              if Motor2levB == 1:
                UpdateMotor2Steps(1) 
          elif gpio == Motor2HallB and Motor2levB == 1:
              if Motor2levA == 1:
                UpdateMotor2Steps(-1)

def ReadGPIOInputs():
    global ser
    global DigIn0
    global DigIn1
    global DigIn2
    global DigIn3
    global DigIn4
    global DigIn5

    GPIO0 =  GPIO.input(DigIn0)    
    GPIO1 =  GPIO.input(DigIn1)    
    GPIO2 =  GPIO.input(DigIn2)    
    GPIO3 =  GPIO.input(DigIn3)    
    GPIO4 =  GPIO.input(DigIn4)    
    GPIO5 =  GPIO.input(DigIn5)  
    GPIO0 = 1 
    CurrentValues = 'IO0: {}  IO1:  {} IO2:  {} IO3: {}  IO4: {} IO5: {}'.format(GPIO0, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5)
    ser.writelines(CurrentValues) 

def SetGPIOOutputs(GPIOPin, GPIOState):
    global DigOut0
    global DigOut1
    global DigOut2
    global DigOut3

    GPIOSetting = GPIO.LOW
    if GPIOState == 1:
        GPIOState = GPIO.HIGH

    if GPIOPin == 0:
        GPIO.output(DigOut0,GPIOState)
    if GPIOPin == 1:
        GPIO.output(DigOut1,GPIOState)
    if GPIOPin == 2:
        GPIO.output(DigOut2,GPIOState)
    if GPIOPin == 3:
        GPIO.output(DigOut3,GPIOState)                

def SetReverse():
    global ReverseEngine
    # set the motor in reverse 
    if ReverseEngine == 1:
       GPIO.output(Motor1Dir,GPIO.LOW)
    else:
       GPIO.output(Motor1Dir,GPIO.HIGH)

def SetReverse2():
    global ZReverseEngine
    # set the motor in reverse 
    if ZReverseEngine == 1:
       GPIO.output(Motor2Dir,GPIO.LOW)
    else:
       GPIO.output(Motor2Dir,GPIO.HIGH)

def SetForward():
    global ReverseEngine
    global Motor1Dir

    # set the motor in forward to find the home position
    if ReverseEngine == 1:
       GPIO.output(Motor1Dir,GPIO.HIGH)
    else:
       GPIO.output(Motor1Dir,GPIO.LOW)

def SetForward2():
    global ZReverseEngine
    global Motor2Dir

    # set the motor in forward to find the home position
    if ZReverseEngine == 1:
       GPIO.output(Motor2Dir,GPIO.HIGH)
    else:
       GPIO.output(Motor2Dir,GPIO.LOW)

def UpdateMotor2Steps(way):
    global Motor2Pos
    global Motor2State
    global StepsToStop2
    global Motor2Direction

    Motor2Pos += way
    if way == 1:
        Motor2Direction = 1
    else:
        Motor2Direction = 0

    print("2 - ", Motor2Pos, StepsToStop2)
    if StepsToStop2 == Motor2Pos: # and Motor2State == 2:
       setDutyMotor2(0) 
       MovingMotor2 = 0
       Motor2State = 0
    if Motor2Pos >= 472 or Motor2Pos <= -472:
       print("Limit 2 detected ", Motor2Pos)
       MovingMotor2 = 0
       setDutyMotor2(0) 
       Motor2State = 0


def FindNegativeMotor1():
    global Motor1Pos
    global Limit1NegReached
    global Motor1InMotion
    global Motor1Negative
    global Motor1NotMoving
    global PreviousMotor1Pos
    global StepsToStop
    global ReverseEngine

    print("Finding Motor 1 Negative limit")
    SetForward()

    Motor1Pos = 0
    if ReverseEngine == 0:
       StepsToStop = -472
    else:
       StepsToStop = 472
    setDutyMotor1(100)  
    time.sleep(0.02)

    while Limit1NegReached == 0:
       time.sleep(0.002)
       if Motor1Pos == PreviousMotor1Pos:
          Motor1NotMoving +=  1
       else:
          PreviousMotor1Pos = Motor1Pos
          Motor1NotMoving = 0

       if Motor1NotMoving >= 1000:
          print("Limit 1 detection. Motor not turning")
          setDutyMotor1(0)
          return 
       if Motor1Pos >= 472 or Motor1Pos <= -472:
          print("Error - could not find home")
          Motor1Negative = 0
          Motor1Pos = 0 # Reset the position to 0. MDM Need to review this 
          setDutyMotor1(0) 
          return
      # if Motor1InMotion == False:
      #    CheckForMotion += 1
      # if CheckForMotion == 100:
      #    setDutyMotor1(0) 
      #    print("Motor Error")
      #    return
    Motor1Negative = 0
    Motor1Pos = 0 # 117  # this is east
    print("Found motor 1 Negative Limit Switch", Motor1Pos, Motor1Negative)
    setDutyMotor1(0) 
    Limit1NegReached = 0
            
def FindNegativeMotor2():
    global Motor2Pos
    global Limit2NegReached
    global Motor2InMotion
    global Motor2Negative
    global Motor2NotMoving
    global PreviousMotor2Pos
    global StepsToStop2
    global ZReverseEngine

    print("Finding Negative limit Motor 2")
    SetForward2()

    Motor2Pos = 0
    if ZReverseEngine == 0:
       StepsToStop2 = -472
    else:
       StepsToStop2 = 472
    setDutyMotor2(100)  
    print("Find 2 starting")
    while Limit2NegReached == 0:
       time.sleep(0.002)
       if Motor2Pos == PreviousMotor2Pos:
          Motor2NotMoving +=  1
       else:
          PreviousMotor2Pos = Motor2Pos
          Motor2NotMoving = 0
       if Motor2Pos >= 472 or Motor2Pos <= -472:
          print("Error - could not find home")
          setDutyMotor2(0) 
          Motor2Negative = 0
          Motor2Pos = 0 # 117  # this is east
          print("Find 2 ending")
          return
      # if Motor1InMotion == False:
      #    CheckForMotion += 1
      # if CheckForMotion == 100:
      #    setDutyMotor1(0) 
      #    print("Motor Error")
      #    return
    Motor2Negative = 0
    Motor2Pos = 0 # 117  # this is east
    print("Found motor 2 Negative Limit Switch", Motor1Pos, Motor1Negative)
    setDutyMotor2(0) 
    Limit2NegReached = 0
            

def FindPositiveMotor1():
    global Limit1PosReached
    global Motor1Pos
    global Motor1Positive
    global Motor1InMotion

    print("Finding Positive limit ")
    SetReverse()

    Motor1Pos = 0
    setDutyMotor1(100)  
    time.sleep(0.02)

    while Limit1PosReached == 0:
       time.sleep(0.02)
       # we may not catch a count so check if this over
       if Motor1Pos >= 472 or Motor1Pos <= -472:
          setDutyMotor1(0) 
          print("Error - could not find home")
          return
       #if Motor1InMotion == False:
       #   CheckForMotion += 1
       #if CheckForMotion == 100:
       #   print("Motor Error")
       #   setDutyMotor1(0) 
       #   return
    Motor1Positive = Motor1Pos
    print("Found motor 1 Positive Limit Switch", Motor1Pos, Motor1Positive)
    setDutyMotor1(0) 
    Limit1PosReached = 0


def FindPositiveMotor2():
    global Limit2PosReached
    global Motor2Pos
    global Motor2Positive
    global Motor2InMotion

    print("Finding motor 2 positive limit ")
    SetReverse2()

    Motor2Pos = 0
    setDutyMotor2(100)  
    time.sleep(0.02)

    while Limit2PosReached == 0:
       time.sleep(0.02)
       # we may not catch a count so check if this over
       if Motor2Pos >= 472 or Motor2Pos <= -472:
          setDutyMotor2(0) 
          print("Error - could not find home")
          return
       #if Motor1InMotion == False:
       #   CheckForMotion += 1
       #if CheckForMotion == 100:
       #   print("Motor Error")
       #   setDutyMotor1(0) 
       #   return

    print("Found motor 2 Positive Limit Switch", Motor2Pos, Motor2Positive)
    setDutyMotor2(0) 

def FindHomeMotor2():
    global Motor2Pos
    print("Homing")
    # set the motor in reverse to find the home position
    GPIO.output(Motor2Dir,GPIO.HIGH)
    #time.sleep(0.02)
    Motor2Pos = 0
    setDutyMotor2(30) 
    time.sleep(0.02)
    Limit_A = GPIO.input(LimitSwitch2)

    print("Home start", Limit_A)
    while Limit_A == 0:
       Limit_A = GPIO.input(LimitSwitch2)
       print("Home cur", Limit_A,Motor2Pos)
       if Motor2Pos == 472 or Motor2Pos == -472:
          print("Error - could not find home")
          return
    print("Found Home")
    Motor2Pos = 117  # this is east

def MoveToPosition(DesiredPosition):
    global ReverseEngine

    ActualPosition = DesiredPosition * 1.3111 # convert to ticks 
    NewPosition = int(ActualPosition)
    if ReverseEngine == 0:
       #NewPosition = NewPosition * -1
       print("Position 1 updated: ", NewPosition)
    print("Motor 1 move: ", NewPosition, Motor1Pos)
    if NewPosition > Motor1Pos: # + 2:
        print("Moving  1 forward from ", Motor1Pos, " To ", NewPosition)
        Motor1State == 1
        MoveMotor1StepsDirection(SolarConstants.MOTOR_FORWARD,NewPosition, 100) # 20, 25)
    if NewPosition < Motor1Pos: # - 2:
        print("Moving 1 reverse from ", Motor1Pos, " To ", NewPosition)
        Motor1State == 1
        MoveMotor1StepsDirection(SolarConstants.MOTOR_REVERSE,NewPosition, 100) # 20, 25)

def MoveToPosition2(DesiredPosition):
    global ZReverseEngine

    ActualPosition = DesiredPosition * 1.3111 # convert to ticks 
    NewPosition = int(ActualPosition)
    if ZReverseEngine == 0:
    #   NewPosition = NewPosition * -1
       print("Position 2 updated: ", NewPosition)
    print("Motor 2 move: ", NewPosition, Motor2Pos)
    if NewPosition > Motor2Pos: # + 2:
        print("Moving motor 2 forward from ", Motor2Pos, " To ", NewPosition)
        Motor2State == 1
        MoveMotor2StepsDirection(SolarConstants.MOTOR_FORWARD,NewPosition, 100) # 20, 25)
    if NewPosition < Motor2Pos: # - 2:
        print("Moving reverse from ", Motor2Pos, " To ", NewPosition)
        Motor2State == 1
        MoveMotor2StepsDirection(SolarConstants.MOTOR_REVERSE,NewPosition, 100) # 20, 25)


def MoveWest():
    if Motor1Pos < 0:
      NewPosition = MotorWestTick
      MoveMotor1StepsDirection(SolarConstants.MOTOR_REVERSE,NewPosition, 20) #100) #25)
            

######################################################################################
#
# Initialize GPIO pins 
#
#
def gpioInit():
   # global motor1Dir
    GPIO.setwarnings(False)

    GPIO.setmode(GPIO.BOARD)  # Set Pi to use pin number when referencing GPIO pins.
                          # Can use GPIO.setmode(GPIO.BCM) instead to use 
                          # Broadcom SOC channel names.
    GPIO.setup(DigOut0, GPIO.OUT)
    GPIO.setup(DigOut1, GPIO.OUT)
    GPIO.setup(DigOut2, GPIO.OUT)
    GPIO.setup(DigOut3, GPIO.OUT)

    GPIO.setup(DigIn0, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    GPIO.setup(DigIn1, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    GPIO.setup(DigIn2, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    GPIO.setup(DigIn3, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    GPIO.setup(DigIn4, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    GPIO.setup(DigIn5, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    '''
    GPIO.setup(Motor1Dir, GPIO.OUT) # set a port/pin as an output 
    GPIO.setup(Motor1HallA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(Motor1HallB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(Motor1HallA, GPIO.BOTH, callback=Motor1Pulse)
    GPIO.add_event_detect(Motor1HallB, GPIO.BOTH, callback=Motor1Pulse)
    GPIO.setup(LimitSwitch1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(LimitSwitch1n, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LimitSwitch1, GPIO.FALLING, callback=LimitSwitchPulse1)
    GPIO.add_event_detect(LimitSwitch1n, GPIO.FALLING, callback=LimitSwitchPulse1)
    GPIO.output(Motor1Dir,GPIO.HIGH)
    GPIO.setup(Motor2Dir, GPIO.OUT) # set a port/pin as an output 
    GPIO.setup(Motor2HallA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(Motor2HallB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(Motor2HallA, GPIO.BOTH, callback=Motor2Pulse)
    GPIO.add_event_detect(Motor2HallB, GPIO.BOTH, callback=Motor2Pulse)
    GPIO.setup(LimitSwitch2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(LimitSwitch2n, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LimitSwitch2, GPIO.FALLING, callback=LimitSwitchPulse2)
    GPIO.add_event_detect(LimitSwitch2n, GPIO.FALLING, callback=LimitSwitchPulse2)
    GPIO.output(Motor2Dir,GPIO.HIGH)
    '''

########################################################################
#
# This assumes channels numbered 0 to 7
#
def ReadADC(ADCChannel):
    # ADC121C021 address, 0x50(80)
    # Select configuration register, 0x02(02)
    #		0x20(32)	Automatic conversion mode enabled

    GPIO.out(MUXSel0,GPIO.LOW)
    GPIO.out(MUXSel1,GPIO.LOW)      
    # setup the MUX
    if ADCChannel < 4:
        if ADCChannel & 0x01:
            GPIO.out(MUXSel0,GPIO.HIGH)
        if ADCChannel & 0x02:
            GPIO.out(MUXSel1,GPIO.HIGH)            
        GPIO.output(CalEn,GPIO.LOW)
        GPIO.output(DigOut0,GPIO.LOW)      
    else:
        ChannelToSet = ADCChannel-4        
        if ChannelToSet & 0x01:
            GPIO.out(MUXSel0,GPIO.HIGH)
        if ChannelToSet & 0x02:
            GPIO.out(MUXSel1,GPIO.HIGH)   
        GPIO.output(MUXSel2,GPIO.HIGH)
        GPIO.output(CalEn,GPIO.LOW)

    time.sleep(0.5)
    bus.write_byte_data(ADCAddress, 0x02, 0x20)

    time.sleep(0.5)

    # ADC121C021 address, 0x50(80)
    # Read data back from 0x00(00), 2 bytes
    # raw_adc MSB, raw_adc LSB
    data = bus.read_i2c_block_data(0x50, 0x00, 2)

    # Convert the data to 12-bits
    raw_adc = (data[0] & 0x0F) * 256 + data[1]

    # Output data to screen
    print ("Digital Value of Analog Input : %d" %raw_adc)


def TeachMotor1():
    FindHomeMotor1()
    FindPositiveMotor1()


def initPWM():
    global pwmMotor1
    global pwmMotor2
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)  # Set Pi to use pin number when referencing GPIO pins.
                          # Can use GPIO.setmode(GPIO.BCM) instead to use 
                          # Broadcom SOC channel names.

    GPIO.setup(Motor1PWM, GPIO.OUT)  # Set GPIO pin 12 to output mode.
    pwmMotor1 = GPIO.PWM(Motor1PWM, 100)   # Initialize PWM on pwmPin 100Hz frequency
    pwmMotor1.start(0)                      # Start PWM with 0% duty cycle
    GPIO.setup(Motor2PWM, GPIO.OUT)  # Set GPIO pin 12 to output mode.
    pwmMotor2 = GPIO.PWM(Motor2PWM, 100)   # Initialize PWM on pwmPin 100Hz frequency
    pwmMotor2.start(0)                      # Start PWM with 0% duty cycle
    print("pwm init***********\n")

def setDutyMotor1( newdc ):
    global pwmMotor1

    pwmMotor1.ChangeDutyCycle(newdc)
    print("Motor 1 dc: ", newdc)

def setDutyMotor2( newdc ):
    global pwmMotor2

    pwmMotor2.ChangeDutyCycle(newdc)
    print("Motor 2 dc: ", newdc)



def FindLimitSwitches1():
    #FindPositiveMotor1() #mdm test
    FindNegativeMotor1()
    #FindPositiveMotor1()


def FindLimitSwitches2():
    #FindPositiveMotor2() # mdm test
    FindNegativeMotor2()
    #FindPositiveMotor2()

def WriteTheMenu():
    global ser

    ser.writelines("Test software Menu. Enter number to run test.\r\n")
    ser.writelines(" 1. Read gpio input pins.\r\n")
    ser.writelines(" 2. Set gpio output pins.\r\n")

#####################################################################################
# Main function
#####################################################################################

#initPWM()
gpioInit()
#bus = smbus.SMBus(1)    # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
#setDutyMotor1(0)  
#setDutyMotor2(0)  
Motor1Pos = 0
Motor2Pos = 0
### 
# Start the motor control threads
'''
#Motor1Thread = threading.Thread(target=motor1_thread, args=(1,))
#Motor1Thread.daemon = True
#Motor1Thread.start()
#time.sleep(10)
#print("Waiting for find home 1")
#while Motor1WaitFlag == 1:
#    time.sleep(10)
#print("done find home 1")

#Motor2Thread = threading.Thread(target=motor2_thread, args=(1,))
#Motor2Thread.daemon = True
#Motor2Thread.start()
#time.sleep(10) # time to let thread start
#print("Motor threads are running")
'''
ser = serial.Serial(
        port='/dev/ttyS0', #AMA0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
#        timeout=1
)



while True:
    WriteTheMenu()
    x=ser.read()

    if x =='1':
        ser.writelines("Reading gpio imputs.")
        ReadGPIOInputs()
    if x =='2':
        ser.writelines("Enter the pin [0..3]: ")
        ThePin = ser.read()
        ser.writelines("\r\nEnter level [0 for low] [1 for high]:")        
        TheState =  ser.read()
        ser.writelines('\r\nSetting pin {} to {}\r\n'.format(ThePin, TheState))
        SetGPIOOutputs(ThePin, TheState) 
        
    if x =='3':
        ser.writelines("Test 3.")

    print(x)

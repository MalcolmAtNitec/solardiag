#####
#
#   Solar tracker v1
import os
import time
import sys
import keyboard
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
import ConfigFile
#import weatherca as localweather
filepath = '/home/pi/iotparams.txt'
filepath2 = '/var/www/html/configurationSettings.txt'

SMTP_SERVER = 'smtp.gmail.com' #Email Server (don't change!)
SMTP_PORT = 587 #Server Port (don't change!)
GMAIL_USERNAME = 'solarresponse1@gmail.com' # gmail account
GMAIL_PASSWORD = 'solarmonitor'  #gmail password

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

windspeed = 'weatherca --city "Ottawa (Richmond - Metcalfe)" --quantity currentConditions/wind/speed'
currenttemp = 'weatherca --city "Ottawa (Richmond - Metcalfe)" --quantity currentConditions/temperature'
sensor_data = {'Angle1': 0, 'Angle2': 0, 'Angle3': 0, 'Wind': 0, 'Temp': 0, 'ZenithOffset': 0, 'AzimuthOffset':0, 'AParkPosition': 0,'ZParkPosition': 0 }
limit_data = {'AzimuthPSL': 0, 'AzimuthNSL': 0, 'ZenithPSL': 0, 'ZenithNSL':0}
method_data = {'GetPartOnOff': 0, 'getTrackOnOff': 0, 'ZenithParkPosValue': 0, 'getAzimuthParkPosValue': 0, 'NightPosition': 0, 'NightAngle': 0}
request_data1 = {'method': 'getTrackerOnOffValue'}
partoff_data = {'ParkValue': 0}
params_data = {'params': 0}
monitor_data = {'AngleBelow': 1}
THINGSBOARD_HOST = '192.168.2.55'
ACCESS_TOKEN = 'iXdKKkSR0Helg1jci892'
THINGSBOARD_PORT = 1883

#################################################
# GPIO Pins are based on board pins
#
# Motor 1 Azimuth based on pin number on connector
Motor1Dir    =  37
Motor1HallA  =  8
Motor1HallB  =  10
Motor1PWM    =  32
LimitSwitch1 =  12
LimitSwitch1n = 16

# Motor 2 Zenith  based on pin number on connector
Motor2Dir    =  18
Motor2HallA  =  22
Motor2HallB  =  40
Motor2PWM    =  33
LimitSwitch2 =  36
LimitSwitch2n = 38

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

###############################################################################
#
#
#Email Variables

class Emailer:
    def sendmail(self, recipient, subject, content):
          
        #Create Headers
        headers = ["From: " + GMAIL_USERNAME, "Subject: " + subject, "To: " + recipient,
                   "MIME-Version: 1.0", "Content-Type: text/html"]
        headers = "\r\n".join(headers)
  
        #Connect to Gmail Server
        session = smtplib.SMTP(SMTP_SERVER, SMTP_PORT)
        session.ehlo()
        session.starttls()
        session.ehlo()
  
        #Login to Gmail
        session.login(GMAIL_USERNAME, GMAIL_PASSWORD)
  
        #Send Email & Exit
        session.sendmail(GMAIL_USERNAME, recipient, headers + "\r\n\r\n" + content)
        session.quit

##################################################################################
#
#
# The following are the callback routines used to communicate with the thingsboard
# IOT platform
#
def GetAParkPos():
    global AParkPosition
    #params_data['value'] = float(AParkPosition) 
    print(json.dumps(float(AParkPosition)))
    return json.dumps(float(AParkPosition))

def SetAParkPos(value):
    global AParkPosition
    AParkPosition = value 
    OpenAndWriteConfigFile()

def GetZParkPos():
    global ZParkPosition
    #params_data['params'] = float(ZParkPosition) 
    print(ZParkPosition)
    return json.dumps(float(ZParkPosition))

def SetZParkPos(value):
    global ZParkPosition
    ZParkPosition = value 
    OpenAndWriteConfigFile()

def GetLongitude():
    return json.dumps(Longitude)

def SetLongitude(value):
    global Longitude
    Longitude = value

def GetLongitude():
    return json.dumps(Latitude)

def SetLongitude(value):
    global Latitude
    Latitude = value

def GetTimeZone():
    return json.dumps(TimeZone)

def SetTimeZone(value):
    global TimeZone
    TimeZone = value

def GetZenithOffset():
    global ZenithOffset
    print("Zenith offset", ZenithOffset)
    return json.dumps(ZenithOffset)

def GetAzimuthOffset():
    global AzimuthOffset
    print("Azimuth offset", AzimuthOffset)
    return json.dumps(AzimuthOffset)

def SetAzimuthOffset(value):
    global AzimuthOffset
    print("Azimuth offset", value)
    AzimuthOffset = value
    OpenAndWriteConfigFile()

def SetZenithOffset(value):
    global ZenithOffset
    print("Zenith offset", value)
    ZenithOffset = value
    OpenAndWriteConfigFile()

def SetAzimuthPSL(value):
    global AzimuthPSLValue 
    print("Set Azimuth: ", value)
    if value == True:
        AzimuthPSLValue = Motor1Pos / 1.311
        if AzimuthNSLValue < 0:
            AzimuthPSLValue = AzimuthPSLValue * -1
        limit_data['AzimuthPSL'] = str(AzimuthPSLValue)
        client.publish('v1/devices/me/telemetry', json.dumps(limit_data), 1)
        OpenAndWriteConfigFile()

def SetAzimuthNSL(value):
    global AzimuthNSLValue 
    print("Set Azimuth NSL: ", value)
    if value == True:
        print("True")
        AzimuthNSLValue = (Motor1Pos/1.311) 
        if AzimuthNSLValue < 0:
            AzimuthNSLValue = AzimuthNSLValue * -1
        print("AzimuthNSL",  AzimuthNSLValue)
        limit_data['AzimuthNSL'] = str(AzimuthNSLValue)
        client.publish('v1/devices/me/telemetry', json.dumps(limit_data), 1)
        OpenAndWriteConfigFile()

def SetZenithPSL(value):
    global ZenithPSLValue 
    print("Set Zenith PSL: ", value)
    if value == True:
        ZenithPSLValue = (Motor2Pos/1.311) 
        if ZenithPSLValue < 0:
            ZenithPSLValue = ZenithPSLValue * -1
        limit_data['ZenithPSL'] = str(ZenithPSLValue)
        client.publish('v1/devices/me/telemetry', json.dumps(limit_data), 1)
        OpenAndWriteConfigFile()

def SetZenithNSL(value):
    global ZenithNSLValue 
    print("Set Zenith: ", value)
    if value == True:
        ZenithPSLValue = (Motor2Pos/1.311) 
        if ZenithNSLValue < 0:
            ZenithNSLValue = ZenithNSLValue * -1
        limit_data['ZenithNSL'] = str(ZenithNSLValue)
        client.publish('v1/devices/me/telemetry', json.dumps(limit_data), 1)
        OpenAndWriteConfigFile()

def SetNightPosition(value):
    global NightPosition
    print("NightPosition ", value)
    if value == False:
        NightPosition = 0 
    if value == True:
        NightPosition = 1 
    OpenAndWriteConfigFile()

def GetNightPosition ():
    global NightPosition 

    print("Night position: ")
    return json.dumps(NightPosition )

def SetNightAngle(value):
    global NightAngle 
    print("Set Night Angle: ", value)
    NightAngle = value
    OpenAndWriteConfigFile()
        
def GetNightAngle():
    global NightAngle 
    
    print("Night angles ", NightAngle)
    return json.dumps(NightAngle)

def GetZenithPSL():
    global ZenithPSL 
    global ZenithPSLValue 
    print("Zenith get value")
    limit_data['ZenithPSL'] = str(ZenithPSLValue)
    client.publish('v1/devices/me/telemetry', json.dumps(limit_data), 1)
    return json.dumps(ZenithPSLValue) 

def GetAzimuthNSL():
    global AzimuthNSL
    global AzimuthNSLValue
    print("Azimuth get value")
    limit_data['AzimuthNSL'] = str(AzimuthNSLValue)
    client.publish('v1/devices/me/telemetry', json.dumps(limit_data), 1)
    return json.dumps(AzimuthNSL)

def GetAzimuthPSL():
    global AzimuthPSL
    global AzimuthPSLValue
    print("Azimuth get value")
    limit_data['AzimuthPSL'] = str(AzimuthPSLValue)
    client.publish('v1/devices/me/telemetry', json.dumps(limit_data), 1)
    return json.dumps(AzimuthNSL)

def GetZenithNSL():
    global ZenithNSL 
    global ZenithNSLValue 
    print("Zenith get value")
    limit_data['ZenithNSL'] = str(ZenithNSLValue)
    client.publish('v1/devices/me/telemetry', json.dumps(limit_data), 1)
    return json.dumps(ZenithNSLValue)

def GetTrackerOnOff():
    global TrackerOnOff
    return json.dumps(TrackerOnOff)

def SetTrackerOnOff(value):
    global TrackerOnOff
    print("Set tracker on off", value)
    if value == False:
        TrackerOnOff = 0 
    if value == True:
        TrackerOnOff = 1 

def GetParkOnOff():
    global ParkOnOff
    print("Park on off: ")
    return json.dumps(ParkOnOff)

def SetParkOnOff(value):
    global ParkOnOff
    print("ParOnOff ", value)
    if value == False:
        ParkOnOff = 0 
    if value == True:
        ParkOnOff = 1 

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected to Cloud server "+str(rc))
    client.subscribe("v1/devices/me/rpc/request/+",1)
    client.subscribe('v1/devices/me/attributes')
    client.publish('v1/devices/me/request/1', json.dumps(limit_data), 1)
    #client.publish('v1/devices/me/rpc/request/1',json.dumps(request_data1), 1)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
   print("From iOT ", msg.topic+" "+str(msg.payload))
   data = json.loads(msg.payload)
   #print(data)
   if data["method"] == "getAzimuthParkPosValue":
       client.publish(msg.topic.replace('request', 'response'), GetAParkPos(), 1)
   elif data["method"] == "getZenithParkPosValue":
       client.publish(msg.topic.replace('request', 'response'), GetZParkPos(), 1)
   elif data['method'] == 'getLongitude':
       client.publish(msg.topic.replace('request', 'response'), GetLongitude(), 1)
   elif data['method'] == 'getLatitude':
       client.publish(msg.topic.replace('request', 'response'), GetLatitude(), 1)
   elif data['method'] == 'getTimeZone':
       client.publish(msg.topic.replace('request', 'response'), GetTimeZone(), 1)
   elif data['method'] == 'getParkOnOffValue':
       client.publish(msg.topic.replace('request', 'response'), GetParkOnOff(), 1)
   elif data['method'] == 'getNightValue':
       client.publish(msg.topic.replace('request', 'response'), GetNightPosition(), 1)
   elif data['method'] == 'getTrackerOnOffValue':
       client.publish(msg.topic.replace('request', 'response'), GetTrackerOnOff(), 1)
   elif data['method'] == 'getZenithOffsetValue':
       client.publish(msg.topic.replace('request', 'response'), GetZenithOffset(), 1)
   elif data['method'] == 'getStopCollectionAngle':
       client.publish(msg.topic.replace('request', 'response'), GetNightAngle(), 1)
   elif data['method'] == 'getAzimuthOffsetValue':
       client.publish(msg.topic.replace('request', 'response'), GetAzimuthOffset(), 1)
   elif data['method'] == 'getAzimuthPSLValue':
       client.publish(msg.topic.replace('request', 'response'), GetAzimuthPSL(), 1)
   elif data['method'] == 'getAzimuthNSLValue':
       client.publish(msg.topic.replace('request', 'response'), GetAzimuthNSL(), 1)
   elif data['method'] == 'getZenithPSLValue':
       client.publish(msg.topic.replace('request', 'response'), GetZenithPSL(), 1)
   elif data['method'] == 'getZenithNSLValue':
       client.publish(msg.topic.replace('request', 'response'), GetZenithNSL(), 1)
   #
   #
   # Do the set methods
   #
   #
   if data["method"] == "setAzimuthParkPosValue":
       value = data['params']
       SetAParkPos(value)
   elif data["method"] == "setZenithParkPosValue":
       value = data['params']
       SetZParkPos(value)
   elif data['method'] == 'setLongitude':
       value = data['params']
       SetLongitude(value)
   elif data['method'] == 'setLatitude':
       value = data['params']
       SetLatitude()
   elif data['method'] == 'setTimeZone':
       value = data['params']
       SetTimeZone(value)
   elif data['method'] == 'setParkOnOffValue':
       value = data['params']
       SetParkOnOff(value)
   elif data['method'] == 'setNightValue':
       value = data['params']
       SetNightPosition(value)
   elif data['method'] == 'setTrackerOnOffValue':
       value = data['params']
       SetTrackerOnOff(value)
   elif data['method'] == 'setZenithOffsetValue':
       value = data['params']
       SetZenithOffset(value)
   elif data['method'] == 'setAzimuthOffsetValue':
       value = data['params']
       SetAzimuthOffset(value)
   elif data['method'] == 'setAzimuthPSLValue':
       value = data['params']
       SetAzimuthPSL(value)
   elif data['method'] == 'setAzimuthNSLValue':
       value = data['params']
       SetAzimuthNSL(value)
   elif data['method'] == 'setZenithNSLValue':
       value = data['params']
       SetZenithNSL(value)
   elif data['method'] == 'setStopCollectionAngle':
       value = data['params']
       SetNightAngle(value)
   elif data['method'] == 'setZenithPSLValue':
       value = data['params']
       SetZenithPSL(value)

def GetWeatherData():
    global Longitude 
    global Latitude 
    global wind
    global temp
    
    api_key = "71e012e3a5e892106b348d67c66c9a32"
    lat = Latitude 
    lon = Longitude 
    url = "https://api.openweathermap.org/data/2.5/onecall?lat=%s&lon=%s&appid=%s&units=metric" % (lat, lon, api_key)
    print(url)

    response = requests.get(url)
    x = response.json()
    y = x['current']
    print(y['temp'])
    temp = y['temp']
    print("Temperature: ", temp)
    print(y['wind_deg'])
    # check if the key exists
    if 'wind_gust' in y:
        print(y['wind_gust']*3.6)
    print(y['wind_speed']*3.6)
    wind = y['wind_speed']*3.6
    print("Wind spped: ", wind)

    ## need to check for the alertr
    if 'alerts' in x:
        z = x['alerts']
        print(z)
#    data = json.loads(response.text)
#    print(data)

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

################################################################################################
#
# This thread sends data for real-time processing to the WEB page. If connected.
#

def data_thread(name):
    global sensor_data
    # get local machine name
    print("Waitinf for connection to WEB Server")
    host = socket.gethostname()                           
    port = 51719
    connected = False
    #conn = 0
    #addr = 0
    while True:
    # create a socket object
    # print(sensor_data)
        time.sleep(60.0)
        if (not connected):
            try:
                s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind((host, port))  
                s.listen(1)
                conn, addr = s.accept()
                print('********************Connection to: ', conn, addr)                             
                connected = True
            except:
                pass
        else:
            try:
                print(sensor_data)
                conn.send(json.dumps(sensor_data)) 
            except:
                print('********************Disconnected: ')     
                conn.close()                        
                connected = False           
        print("Data thread running")

def web_thread(name):
    global NewDataToRead
    # get local machine name
    host1 = 0
    port1 = 0
    connected = False
    NewDataToRead = False
    host1 = socket.gethostname()                           
    port1 = 51720
    while True:
            time.sleep(30.0)    # create a socket object            try:
            if (not connected):
                try:
                    s2=socket.socket(socket.AF_INET, socket.SOCK_STREAM)                
                    s2.connect((host1, port1))
                    connected = True
                    print("Connected to the web")
                except:
                    pass
            else:
                try:
                    msg =''
                    msg = s2.recv(1024)  
                    print("received", msg)
                    ConfigFile.AssignAzimuthParametrs()
                    ConfigFile.AssignCloudParametrs()
                    ConfigFile.AssignZenithParametrs()
                    ConfigFile.AssignLocationParametrs()
                    ConfigFile.AssignTrackerParametrs()
                    print("Parameters updated")
                except:
                #    print("Client disconnected")
                #    s2=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    connected = False
                    pass
            #if RxNewData == True:

        
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
    CheckForMotion = 0
    Motor1NegReached = 0
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
    CheckForMotion = 0
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
    CheckForMotion = 0
    CurrentMotorPosition = Motor1Pos
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
    CheckForMotion = 0
    CurrentMotorPosition = Motor2Pos
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
    Motor1Positive = Motor2Pos
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
# Read and write configuration parameters
#

def OpenAndReadConfigFileorg():
    global ZenithOffset
    global AzimuthOffset
    global AParkPosition
    global ZParkPosition
    global ZenithNSLValue
    global ZenithPSLValue
    global AzimuthNSLValue
    global AzimuthNSLValue
    global NightAngle
    global NightPosition

    cnt = 1
    with open(filepath) as fp:
       line = fp.readline()
       print("Line {}: {}".format(cnt, line.strip()))
       ZenithOffset = float(line.strip()) 

       line = fp.readline()
       print("Line {}: {}".format(cnt, line.strip()))
       AzimuthOffset =  float(line.strip()) 

       line = fp.readline()
       print("Line {}: {}".format(cnt, line.strip()))
       AParkPosition =  float(line.strip()) 

       line = fp.readline()
       print("Line {}: {}".format(cnt, line.strip()))
       ZParkPosition = float(line.strip()) 

       line = fp.readline()
       print("Line {}: {}".format(cnt, line.strip()))
       AzimuthPSLValue = float(line.strip()) 

       line = fp.readline()
       print("Line {}: {}".format(cnt, line.strip()))
       AzimuthNSLValue = float(line.strip()) 

       line = fp.readline()
       print("Line {}: {}".format(cnt, line.strip()))
       ZenithPSLValue = float(line.strip()) 

       line = fp.readline()
       print("Line {}: {}".format(cnt, line.strip()))
       ZenithNSLValue = float(line.strip()) 

       line = fp.readline()
       print("Line {}: {}".format(cnt, line.strip()))
       NightAngle = float(line.strip()) 

       line = fp.readline()
       print("Line {}: {}".format(cnt, line.strip()))
       NightPosition = float(line.strip()) 

    print(ZenithOffset, AzimuthOffset, AParkPosition, ZParkPosition)
    print(AzimuthPSLValue, AzimuthNSLValue,ZenithPSLValue, ZenithNSLValue)
    fp.close
    limit_data['AzimuthPSL'] = str(AzimuthPSLValue)
    limit_data['AzimuthNSL'] = str(AzimuthNSLValue)
    limit_data['ZenithPSL'] = str(ZenithPSLValue)
    limit_data['ZenithNSL'] = str(ZenithNSLValue)
    limit_data['ZenithOffset'] = str(ZenithOffset)
    limit_data['AzimuthOffset'] = str(AzimuthOffset)
    limit_data['AParkPosition'] = str(AParkPosition)
    limit_data['ZParkPosition'] = str(ZParkPosition)
    limit_data['NightAngle'] = str(NightAngle)
    limit_data['NightPosition'] = str(NightPosition)


def OpenAndReadConfigFile2org():
    global THINGSBOARD_HOST 
    global THINGSBOARD_PORT 
    global ACCESS_TOKEN
    global ReverseEngine 
    global ZReverseEngine 


    cnt = 1
    with open(filepath2) as fp:
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()
       line = fp.readline()


       line = fp.readline()
       print("Line {}: {}".format(cnt, line.strip()))
       ACCESS_TOKEN = line.strip()
       line = fp.readline()
       print("Line {}: {}".format(cnt, line.strip()))
       THINGSBOARD_HOST = line.strip()
       line = fp.readline()
       THINGSBOARD_PORT = int(line.strip())
       line = fp.readline()
       ReverseEngine = int(line.strip())
       line = fp.readline()
       ZReverseEngine = int(line.strip())
    print(ACCESS_TOKEN, THINGSBOARD_HOST, THINGSBOARD_PORT)
    print("Reverse: ", ReverseEngine)
    print("ZReverse: ", ZReverseEngine)
    fp.close

def OpenAndWriteConfigFile():
    global ZenithOffset
    global AzimuthOffset
    global AParkPosition
    global ZParkPosition
    global ZenithNSLValue
    global ZenithPSLValue
    global AzimuthNSLValue
    global AzimuthNSLValue
    global NightAngle
    global NightPosition

    print("Saving parameters")
    with open(filepath, "w") as fp:
       fp.write(str(ZenithOffset) + '\n')
       fp.write(str(AzimuthOffset) + '\n')
       fp.write(str(AParkPosition) + '\n')
       fp.write(str(ZParkPosition) + '\n')
       fp.write(str(AzimuthPSLValue) + '\n')
       fp.write(str(AzimuthNSLValue) + '\n')
       fp.write(str(ZenithPSLValue) + '\n')
       fp.write(str(ZenithNSLValue) + '\n')
       fp.write(str(NightAngle) + '\n')
       fp.write(str(NightPosition) + '\n')
    print("Saving config values: ", ZenithOffset, AzimuthOffset, AParkPosition, ZParkPosition, NightAngle, NightPosition)
    fp.close


######################################################################################
#
# Initialize GPIO pins 
#
#
def gpioInit():
    global motor1Dir
    GPIO.setwarnings(False)

    GPIO.setmode(GPIO.BOARD)  # Set Pi to use pin number when referencing GPIO pins.
                          # Can use GPIO.setmode(GPIO.BCM) instead to use 
                          # Broadcom SOC channel names.
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


#####################################################################################
# Main function
#####################################################################################

# First read the configuration parameters
#
print("Reading configuration files")
ConfigFile.OpenAndReadConfigFile(filepath)
ConfigFile.OpenAndReadConfigFile2(filepath2)


GetWeatherData()
sender = Emailer()
 

############################################
# 
# Connect to the thingsboard iOT platform.
# Connect to ThingsBoard using default MQTT port and 60 seconds keepalive interval
#
# connection to hostname on the port.
client = mqtt.Client()
try:
    client.on_connect = on_connect
    client.on_message = on_message
    print("Connecting to cloud: ", THINGSBOARD_HOST, THINGSBOARD_PORT)
    #client.publish('v1/devices/me/attributes/request/1', "{\"clientKeys\":\"model\"}", 1)
    # Set access token for connection
    client.username_pw_set(ACCESS_TOKEN)
    client.connect(THINGSBOARD_HOST, THINGSBOARD_PORT, 60)   #1883, 60)
    client.loop_start() # connect to thingsboard
except:
    sendTo = 'malcolm@niteccorp.com'
    emailSubject = "Solar panel alert"
    emailContent = "Cannot connect to the cloud"
 
    #Sends email to "sendTo" address with "emailSubject" as the subject and "emailContent" as the email content.
    sender.sendmail(sendTo, emailSubject, emailContent)  

# Start the data monitoring thread
WEBThread = threading.Thread(target=web_thread, args=(1,))
WEBThread.daemon = True
WEBThread.start()

# mdm print("Connecting to weather ")
x = threading.Thread(target=weather_thread, args=(1,))
x.start()

ConnectThread = threading.Thread(target=data_thread, args=(1,))
ConnectThread.daemon = True
ConnectThread.start()

initPWM()
gpioInit()

setDutyMotor1(0)  
setDutyMotor2(0)  
Motor1Pos = 0
Motor2Pos = 0
### 
# Start the motor control threads
Motor1Thread = threading.Thread(target=motor1_thread, args=(1,))
Motor1Thread.daemon = True
Motor1Thread.start()
time.sleep(10)
print("Waiting for find home 1")
while Motor1WaitFlag == 1:
    time.sleep(10)
print("done find home 1")

Motor2Thread = threading.Thread(target=motor2_thread, args=(1,))
Motor2Thread.daemon = True
Motor2Thread.start()
time.sleep(10) # time to let thread start
print("Motor threads are running")

#TeachMotor1()

###
# FindHomeMotor2()
   

# get local machine name
host = socket.gethostname()                           
port = 51717
# create a socket object
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
    s.connect((host, port))                               
except:
    print("Error connecting to sun server")

try:
    while True:
        SolarReadings = []
        # Receive no more than 1024 bytes
        msg = ''    # clear the string
        try:
            msg = s.recv(1024)  
            print(msg.decode('ascii'))

            for item in msg.split() :
                SolarReadings.append(float(item))
        except:
            print("No socket")
            pass

        if NewDataToRead == True:
            #ConfigFile.AssignAzimuthParametrs()
            #ConfigFile.AssignCloudParametrs()
                #ConfigFile.AssignZenithParametrs()
                #ConfigFile.AssignLocationParametrs()
            ConfigFile.AssignTrackerParametrs()                    
            print(msg, "!!updated data")#, msg)
            NewDataToRead = False

        # do we require this print(GPIO.input(Motor2HallA),GPIO.input(Motor2HallA) )
        if Motor1NotMoving >= 1000:
            # report error to user
            print("Error: Motor1 Not turning")
        if Motor2NotMoving >= 1000:
            # report error to user
            print("Error: Motor2 Not turning")

        humidity = 38.2 #round(humidity, 2)
        temperature = 38.2 #round(temperature, 2)
        #print(temp)
        #print(wind)
        #print(u"Temperature: {:g}\u00b0C, Humidity: {:g}%".format(temperature, humidity))
        #print("Azimuth:", SolarReadings[0], ", ", SolarReadings[1])
        #print(u"Elevation: {:g}\u00b0C, Azimuth: {:g}%".format(SolarReadings[0], SolarReadings[1]))
        # Sending sensir and information to ThingsBoard
        sensor_data['Angle1'] = float(SolarReadings[0])# 38.3 
        sensor_data['Angle2'] = float(SolarReadings[1]) #180.2 
        sensor_data['Angle3'] = float(SolarReadings[2]) #180.2 
        sensor_data['Wind'] =  wind 
        sensor_data['Temp'] = temp 
        client.publish('v1/devices/me/telemetry', json.dumps(sensor_data), 1)

        if TrackerOnOff == 1:
            AzimuthAngle =  float(SolarReadings[2])
            ZenithAngle = float(SolarReadings[1])
            print("Angles: ", AzimuthAngle, ZenithAngle, NightAngle)
            if AzimuthAngle < NightAngle:
                MoveToNightPosition = 1
                   
        #elif ParkOnOff == 1:
        #    MoveToPosition(AParkPosition)
        #    MoveToPosition2(ZParkPosition)
        #if keyboard.read_key() == "q":
        #    print("entered q - Stopping the motors")
        #    setDutyMotor1(0) 
        #    setDutyMotor2(0) 
        #    client.disconnect()
        #    Motor1Thread.join()
        #    x.join()
        #    s.join()
        #    sys.exit()


        
except KeyboardInterrupt:
    print("Stopping the motors")
    setDutyMotor1(0) 
    setDutyMotor2(0) 
    #client.disconnect()
    #print("Stopping motor threads")
    #Motor1Thread.join()
    print("Stopping motor threads")
    #Motor2Thread.join()
    print("Stopping other threads")
    x.join()
    s.join()
    s2.join()
    s2.close()
    s.close()
     
    print("Exiting application")
    client.loop_stop()
    pass

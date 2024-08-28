# -*- coding: utf-8 -*-
"""
Created on Mon Mar 18 16:27:55 2024

@author: chris
ΑΒΓΔΕΖΗΘΙΚΛΜΝΞΟΠΡΣΤΥΦΧΨΩαβγδεζηθικλμνξοπρσςτυφχψωάέήϊίόύϋώΆΈΉΊΌΎΏ±≥≤ΪΫ÷≈°√ⁿ²ˑ
"""

import numpy as np
import matplotlib.pyplot as plt
#pip install pyserial for the serial port stuff (does not work with pip install serial)
import serial
#import serial.tools.list_ports
import time
from datetime import datetime

##########################################################################
###   Utility stuff for STM32F746 discovery board with ImuTest01 code
###     
##########################################################################
def findPort(VID,PID,num=1):
    '''
    PID is defined below for each board.
    If there are multiple matching VID:PID, then chose the one given by num
    Using the VID is assigned to ST Micro, so potentially there could be a conflict
    F476 Discovery board:  VID:PID=0483:5740
    '''
    n=0
    name = ''
    for port in serial.tools.list_ports.comports():
        if (port.vid==VID) and (port.pid==PID):
            n=n+1
            if (n==num):
                name = port.name
    return name

def readReg(reg):
    '''
    Read a register on one of Marc's STM32 boards.
    Write a string like "r0"
    Read a string like "r0=1" then parse and return the number at the end
    '''
    ser.write(b'r'+bytes(hex(reg)[2:],'ascii')+b'\n')
    text = ser.readline()
    iStart = 1+text.index(b'=')
    iEnd = text.index(b'\n')
    return int(text[iStart:iEnd],16)
     
def writeReg(reg,val):
    '''
    Write a register on one of Marc's STM32 boards. 
    The format of the string is something like "r12=5a"
    '''
    ser.write(b'w'+bytes(hex(reg)[2:],'ascii')+b'='+bytes(hex(val)[2:],'ascii')+b'\n')
    ser.readline() #need to read response so it doesn't fill up buffer

def toSigned16(n):
    '''
    Converts a 16-bit number from unsigned (range 0~65535) to signed (range -32768~32767). 
    '''
    n = n & 0xffff
    return (n ^ 0x8000) - 0x8000

def toSigned24(n):
    '''
    Converts a 24-bit number from unsigned (range 0~65535) to signed (range -32768~32767). 
    '''
    n = n & 0xffffff
    return (n ^ 0x800000) - 0x800000

regDict = {
        'RegFirmWareVersion' : 0,      
        'RegUniqueID' : 1,                
        'RegTick' : 2,                     
        'RegAdcTemp' : 3,                 
        'RegAdcRef' : 4,
        'RegPeltierTempHeatsinkAdc' : 5,
        'RegPeltierTempHeatsinkCelsius' : 6,
        'RegPeltierTempHotsideAdc' : 7,
        'RegPeltierTempHotsideCelsius' : 8,
        'RegPeltierPWMDutyCycle' : 9
        }
#regDict['']


##########################################################################
###   Try reading some stuff that is internal to the 'F746
###     
##########################################################################
#ser = serial.Serial(findPort(0x0483,0x5740), timeout=1)
ser = serial.Serial('COM12', baudrate=115200, timeout=1)

#writeReg(regDict['RegEncoderCwSteps'], 0)
#writeReg(regDict['RegEncoderCCwSteps'], 0)  

#print('Firmware Version = ', readReg(regDict['RegFirmWareVersion']))
#print('Unique board ID = ', readReg(regDict['RegUniqueID']))
#print('Timer Tick val = ', readReg(regDict['RegTick'])) 

tempAdc = readReg(regDict['RegAdcTemp'])
#print(f'Temp ADC = {tempAdc}, which is a temperature of {25+(tempAdc*3.3/4095-0.76)/0.025:.2f}°C') 

refAdc = readReg(regDict['RegAdcRef'])
#print(f'internal reference ADC = {refAdc}, which is a voltage of {refAdc*3.3/4095:.3f}V') 

tm1 = readReg(regDict['RegTick'])
time.sleep(1)
tm2 = readReg(regDict['RegTick'])
#print(f'time Difference = {tm2 - tm1} mS')



####################################################################################################################################################
###  
####################################################################################################################################################


writeReg(regDict['RegPeltierPWMDutyCycle'], 55)
adc = readReg(regDict['RegPeltierTempHeatsinkAdc'])
cel = readReg(regDict['RegPeltierTempHeatsinkCelsius'])
print(adc)
print(cel)

















##########################################################################
###   Close the serial connection
###   
##########################################################################

ser.close()
    



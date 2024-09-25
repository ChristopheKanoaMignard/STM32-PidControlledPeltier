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
import csv
import statistics
import os

##########################################################################
###   Utility stuff for communicating with STM32 boards.
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
    Read a register on a STM32 board.
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
    Write a register on a STM32 boards. 
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

def convertAdcToCels(adc):
    cels = (1. / (1. / 3650 * -np.log(4095.00000001 / adc - 1) + 1. / 298)) - 273
    return cels

#This expects rawData to be a 2D array.
#Any data point that is cutoffJump greater than the previous data point will be replaced with the average of all data points within n indecies. 
def interpolate2dData(rawData, cutoffJump, n):         
    interpData = np.empty_like(rawData)
    np.copyto(interpData, rawData)
   
    for i in range(len(rawData)):
        for j in range(n, len(rawData[0])-n):
            if ( (abs(interpData[i][j] - interpData[i][j-1])) > cutoffJump):    #If current data point jumps cutoffJump above previous data point, interpolate it. 
                interpData[i][j] = np.average([interpData[i][(j-n) : j], interpData[i][j+1 : j+n+1]])   #Average the n previous and the n proceding data points
                
    return interpData
                                      
regDict = {
        'RegFirmWareVersion'                  :0,
        'RegUniqueID'                        :1, 
        'RegTick'                             :2, 
        'RegAdcTemp'                        :3, 
        'RegAdcRef'                         :4, 
    	'RegPeltierTempColdsideAdc'        :5,
    	'RegPeltierTempColdsideCelsius'	   :6,
    	'RegPeltierTempHotsideAdc'         :7,
    	'RegPeltierTempHotsideCelsius'     :8,
    	'RegPeltierPWMDutyCycle'           :9, 
    	'RegPeltierReverseMode'				:10,
    	'RegPeltierBangBangTempTargetAdc'	    :11,
    	'RegPeltierBangBangTempAllowanceAdc'	:12, 
    	'RegPeltierPidSetpointAdc'			:13, 
    	'RegPeltierPidDt'					   :14, 
    	'RegPeltierPidKp'					   :15,
    	'RegPeltierPidKi'					   :16, 
    	'RegPeltierPidKd'					   :17, 
    	'RegPeltierPidManipulatedVariable' :18,	
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


















####################################################################################################################################################
###  Plot desired data
####################################################################################################################################################

#Plotting flags
noiseCharacteristics = False
parameterTuningKp = False
parameterTuningKi_Kp1000 = True

#Plotting consistent variables
dataPeriod = 0.1        #By default, gui reads data at 10Hz, ie a period of 0.1 sec

if parameterTuningKi_Kp1000:
    ############################### Setup data arrays ###############################
    boolMinLen = 0
    boolMaxLen = 1
    
    # Generating fileNames this way looks nicer, but I prefer being able to control the order of the list.
    #directory = 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/'
    #fileNames = []  
    #for files in os.listdir(directory):
        #fileNames.append(directory + files)
    
    #fileName_IntegralSumMax_Ki = ...
    fileNames = ['C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax5000_Ki2.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax5000_Ki10.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax10000_Ki10.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax15000_Ki10.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax20000_Ki10.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax25000_Ki10.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax25000_Ki100.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax25000_Ki500.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax25000_Ki1000.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax25000_Ki2000.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax30000_Ki100.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax30000_Ki500.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax30000_Ki1000.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax30000_Ki2000.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax35000_Ki10.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax35000_Ki100.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax35000_Ki500.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax35000_Ki1000.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax40000_Ki10.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax40000_Ki100.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax40000_Ki500.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2200_DutyMax56_Kp1000_IMax40000_Ki1000.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2500_DutyMax75_Kp1000_IMax35000_Ki500.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2500_DutyMax75_Kp1000_IMax45000_Ki500.csv', 
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2500_DutyMax75_Kp1000_IMax55000_Ki500.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2500_DutyMax75_Kp1000_IMax65000_Ki500.csv', 
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To1400_DutyMax75_Kp1000_IMax35000_Ki100.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To1400_DutyMax75_Kp1000_IMax35000_Ki500.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To1400_DutyMax75_Kp1000_IMax35000_Ki1000.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To1400_DutyMax75_Kp1000_IMax40000_Ki100.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To1400_DutyMax75_Kp1000_IMax40000_Ki500.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To1400_DutyMax75_Kp1000_IMax40000_Ki1000.csv',
                 ]

    labels = fileNames.copy()                   #Gets usefully filled inside for loop
    dummyData = []                              #np doesn't support jagged arrays, so first populate a temporary array with the jagged values, then trim them later
    minLen = np.inf                             #used to trim jagged array to the smallest list inside the list of lists
    maxLen = 0                                  #used to expand jagged array to the largest list inside the list of lists
    for i in range(len(fileNames)):
        labels[i] = fileNames[i][136:-4]        #Store only the useful identifier characters at the end of the file address from the 113th char to 4 chars before the end
        dummyData.append(np.genfromtxt(fileNames[i], delimiter= ',', skip_header=1, usecols=(0)))      #Temperature of peltier coldside is stored in column 0; skip 1 row which are column headers
        minLen = min(minLen, len(dummyData[i]))
        maxLen = max(maxLen, len(dummyData[i]))
       
    targetTempAdcVal = np.array(np.genfromtxt(fileNames[0], delimiter= ',', skip_header=1, usecols=(4)))[0]     #SetPoint of peltier coldside is stored in column 4 (5th); skip 1 row which are column headers
    targetTempExtremeAdcVal = np.array(np.genfromtxt(fileNames[23], delimiter= ',', skip_header=1, usecols=(4)))[0]     #SetPoint of peltier coldside is stored in column 4 (5th); skip 1 row which are column headers
    targetTempHotAdcVal = np.array(np.genfromtxt(fileNames[24], delimiter= ',', skip_header=1, usecols=(4)))[0]
    
    if boolMinLen:
        tempsAdc = np.empty((len(dummyData), minLen))       #Initialize np array with proper lengths
        tempsCels = np.empty((len(dummyData), minLen))      #Initialize np array with proper lengths
        targetTemp = np.ones(minLen) * convertAdcToCels(targetTempAdcVal)       #Array of length minLen with every element equal to SetPoint in *C
        targetTempExtreme = np.ones(minLen) * convertAdcToCels(targetTempExtremeAdcVal)       #Array of length minLen with every element equal to SetPoint in *C
        targetTempHot = np.ones(minLen) * convertAdcToCels(targetTempHotAdcVal)       #Array of length minLen with every element equal to SetPoint in *C
        dataTime = 0.1 * np.arange(0, minLen)                                   #Data was read by gui at 10Hz, ie a period of 0.1 sec
        
        for i in range(len(dummyData)):
            tempsAdc[i] = dummyData[i][:minLen]             #Use minLen to cut all longer data sets to the minimum
            
          
    if boolMaxLen:
        tempsAdc = np.empty((len(dummyData), maxLen))       #Initialize np array with proper lengths
        tempsCels = np.empty((len(dummyData), maxLen))      #Initialize np array with proper lengths
        targetTemp = np.ones(maxLen) * convertAdcToCels(targetTempAdcVal)       #Array of length maxLen with every element equal to SetPoint in *C
        targetTempExtreme = np.ones(maxLen) * convertAdcToCels(targetTempExtremeAdcVal)       #Array of length maxLen with every element equal to SetPoint in *C
        targetTempHot = np.ones(maxLen) * convertAdcToCels(targetTempHotAdcVal)       #Array of length maxLen with every element equal to SetPoint in *C
        dataTime = np.arange(0, maxLen) * dataPeriod                            #Data was read by gui at 10Hz, ie a period of 0.1 sec
    
        for i in range(len(dummyData)):
            tempsAdc[i] = np.pad(dummyData[i], 
                                 pad_width = (0, (maxLen - len(dummyData[i]))), constant_values = (None, None))  #Use maxLen to pad empty values to the end of all data sets shorter than the maximum

    tempsCels = convertAdcToCels(tempsAdc)                                  #Will graph temperature measurments in *C
    
    
    
    ############################### Graph data ###############################
    
    dataToPlot = interpolate2dData(tempsCels, cutoffJump = 0.1, n=3)   #Interpolate any data point that jumps 0.1*C in one dataPeriod using 3 previous and 3 proceding data points. 
    
    
    
    ### Graph all data points on their own figure ###
    
    for i in range(len(dataToPlot)):
        figure = plt.figure()
        plt.title("PID Controller Ki Tuning, data set #"+(str)(i))
        plt.plot(dataTime, dataToPlot[i], label = labels[i])
        plt.plot(dataTime, targetTemp, label = "Set Point", color='b', linestyle ='dashed')
        plt.xlabel('Time (s)')
        plt.ylabel('Temperature (°C)')
        plt.xlim(dataTime[0], dataTime[-1])
        plt.ylim(20, 30)
        plt.legend(bbox_to_anchor=(1, 1), borderaxespad=0)
        plt.show()
        
        
        
    ### Graph multiple data sets on a single figure. Plot any set whose index is True in the multiPlotMask ###
    
    multiPlotMask = np.zeros(len(dataToPlot), dtype=bool)
    multiPlotMask[6] = multiPlotMask[7] = multiPlotMask[8] = multiPlotMask[9] = True
    
    figure = plt.figure()
    plt.title("PID Controller Ki Tuning")
    
    for i in range(len(dataToPlot)):
        if multiPlotMask[i]:                                        #Plot data set if its index in the dataToPlot corresponds to a True value for that same index in multiPlotMask
            plt.plot(dataTime, dataToPlot[i], label = labels[i])
            
    plt.plot(dataTime, targetTemp, label = "Set Point", color='b', linestyle ='dashed')
    plt.xlabel('Time (s)')
    plt.ylabel('Temperature (°C)')
    plt.xlim(20, 50)
    plt.ylim(21, 22)
    
    plt.legend()
    #plt.legend(bbox_to_anchor=(1, 1), borderaxespad=0)
    plt.show()
    
    

    ### Graph data on 4 figures in a grid. ###
    
    effectChangingIMax = np.array(range(2, 6))
    iMax2500_KiRange = np.array(range(6, 10))
    iMax3000_KiRange = np.array(range(10, 14))
    iMax3500_KiRange = np.array(range(14, 18))
    iMax4000_KiRange = np.array(range(18, 22))
    #unmasked = iMax4000_KiRange
    unmasked = [effectChangingIMax, iMax2500_KiRange, iMax3000_KiRange, iMax3500_KiRange, iMax4000_KiRange]
    
    k = 0
    l = 0
    for l in range(len(unmasked)):
        figure, axis = plt.subplots(2, 2, layout="constrained")
        xlim = [20, 50]
        ylim = [21, 22]
        #xlim = [0, 50]
        #ylim = [21, 29]
        k = 0
        for i in range(2):
            for j in range(2): 
                axis[i, j].plot(dataTime,  dataToPlot[unmasked[l][k]])
                axis[i, j].set_title(labels[unmasked[l][k]])

                axis[i, j].plot(dataTime,  targetTemp)
                axis[i, j].set_xlim(xlim)
                axis[i, j].set_ylim(ylim)
                axis[1, j].set_xlabel("Time (s)")
                axis[i, 0].set_ylabel("Temp (°C)")
                axis[0, j].set_xticklabels([])
                axis[i, 1].set_yticklabels([])
                k+=1                           
    
        figure.suptitle("Tuning Ki over various Integration Maximums")
        #text = ("Discussion")
        #figure.text(.5, -.1, text, ha='center')
        plt.show()
    
    
    
    ### Graph extreme cooling ###
    
    
    multiPlotMask = np.zeros(len(dataToPlot), dtype=bool)
    multiPlotMask[22] = multiPlotMask[23] = True
    
    figure = plt.figure()
    
    
    plt.title("PI Controller: Minimum Temperature")
    
    for i in range(len(dataToPlot)):
        if multiPlotMask[i]:                                        #Plot data set if its index in the dataToPlot corresponds to a True value for that same index in multiPlotMask
            plt.plot(dataTime, dataToPlot[i], label = labels[i])


    plt.plot(dataTime, targetTempExtreme, label = "Set Point", color='b', linestyle ='dashed')
    plt.xlabel('Time (s)')
    plt.ylabel('Temperature (°C)')
    plt.xlim(75, 115)
    plt.ylim(14, 16)
    
    plt.legend()
    #plt.legend(bbox_to_anchor=(1, 1), borderaxespad=0)
    plt.show()
    
    
    
    ### Heating ###

    figure, axis = plt.subplots(3, 2, layout="constrained")
    xlim = [25, 50]
    ylim = [39, 43]
    k = 0
    
    heatingStartIndex = fileNames.index('C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To1400_DutyMax75_Kp1000_IMax35000_Ki100.csv')

    for j in range(2):
        for i in range(3):   
            axis[i, j].plot(dataTime, dataToPlot[heatingStartIndex + k])
            axis[i, j].plot(dataTime, targetTempHot)
            axis[i, j].set_title(labels[heatingStartIndex + k])
            axis[i, j].set_xlim(xlim)
            axis[i, j].set_ylim(ylim)
            axis[2, j].set_xlabel("Time (s)")
            axis[i, 0].set_ylabel("Temp (°C)")
            k+=1
        
    #text = ("Discussion")

    figure.suptitle("PI Controller: Parameter tuning for heating")
    #figure.text(.5, -.1, text, ha='center')
    plt.show()
    
    
    
    ### Extreme cooling ###
    
    figure, axis = plt.subplots(2, 2, layout="constrained")
    xlim = [80, 150]
    ylim = [13, 16]
    k = 0
    
    extremeCoolingStartIndex = fileNames.index('C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PIController_SP2000To2500_DutyMax75_Kp1000_IMax35000_Ki500.csv')
    
    for i in range(2):
        for j in range(2):   
            axis[i, j].plot(dataTime, dataToPlot[extremeCoolingStartIndex + k])
            axis[i, j].plot(dataTime, targetTempHot)
            axis[i, j].set_title(labels[extremeCoolingStartIndex + k])
            axis[i, j].set_xlim(xlim)
            axis[i, j].set_ylim(ylim)
            axis[1, j].set_xlabel("Time (s)")
            axis[i, 0].set_ylabel("Temp (°C)")
            k+=1
        

    #text = ("Discussion")

    figure.suptitle("PI Controller: Tuning integral sum max for extreme cooling")
    #figure.text(.5, -.1, text, ha='center')
    plt.show()









if parameterTuningKp:
    fileName20 = 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PController_Kp20.csv'
    fileName100 = 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PController_Kp100.csv'
    fileName200 = 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PController_Kp200.csv'
    fileName500 = 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PController_Kp500.csv'
    fileName1000 = 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PController_Kp1000.csv'
    fileName2000 = 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PController_Kp2000.csv'
    #fileTuningKp20 = open(fileName20, 'r')
    #fileTuningKp100 = open(fileName100, 'r')
    #fileTuningKp200 = open(fileName200, 'r')
    #fileTuningKp500 = open(fileName500, 'r')
    #fileTuningKp1000 = open(fileName1000, 'r')
    #fileTuningKp2000 = open(fileName2000, 'r')
    #csvTuningKp20 = csv.DictReader(fileTuningKp20)
    #csvTuningKp100 = csv.DictReader(fileTuningKp100)
    #csvTuningKp200 = csv.DictReader(fileTuningKp200)
    #csvTuningKp500 = csv.DictReader(fileTuningKp500)
    #csvTuningKp1000 = csv.DictReader(fileTuningKp1000)
    #csvTuningKp2000 = csv.DictReader(fileTuningKp2000)
    
    temps20 = np.genfromtxt(fileName20, delimiter= ',', skip_header= 1, usecols=(0))
    temps100 = np.genfromtxt(fileName100, delimiter= ',', skip_header= 1, usecols=(0))
    temps200 = np.genfromtxt(fileName200, delimiter= ',', skip_header= 1, usecols=(0))
    tempsCels20 = (1. / (1. / 3650 * -np.log(4095.00000001 / temps20 - 1) + 1. / 298)) - 273
    tempsCels100 = (1. / (1. / 3650 * -np.log(4095.00000001 / temps100 - 1) + 1. / 298)) - 273
    tempsCels200 = (1. / (1. / 3650 * -np.log(4095.00000001 / temps200 - 1) + 1. / 298)) - 273
    
    time20 = 0.1 * np.arange(0, len(temps20))       #Data taken at 10Hz. ADC values are sampled&averaged at 33.3Hz
    time100 = 0.1 * np.arange(0, len(temps100))       #Data taken at 10Hz. ADC values are sampled&averaged at 33.3Hz
    time200 = 0.1 * np.arange(0, len(temps200))       #Data taken at 10Hz. ADC values are sampled&averaged at 33.3Hz
    
    figure, axis = plt.subplots(2, 2, layout="constrained")
    xlim = [0, 200]
    ylim = [2185, 2197]
    ylimCels = [15, 30]
    
    axis[0, 0].plot(time20,  tempsCels20)
    axis[0, 0].set_title("Kp=20")
    axis[0, 0].set_xlim(xlim)
    axis[0, 0].set_ylim(ylimCels)
    axis[0, 0].set_xlabel("Time (s)")
    axis[0, 0].set_ylabel("Temp (°C)")
    
    axis[0, 1].plot(time100,  tempsCels100)
    axis[0, 1].set_title("Kp=100")
    axis[0, 1].set_xlim(xlim)
    axis[0, 1].set_ylim(ylimCels)
    axis[0, 1].set_xlabel("Time (s)")
    axis[0, 1].set_ylabel("Temp (°C)")
    

    
    fileNames = ['C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PController_Kp20.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PController_Kp100.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PController_Kp200.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PController_Kp500.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PController_Kp1000.csv',
                 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Parameter Tuning Single Loop/Peltier_PController_Kp2000.csv' ]
    
    dummyData = []      #np doesn't support jagged arrays, so first populate a temporary array with the jagged values, then trim them later
    minLen = np.inf     #used to trim jagged array to the smallest list inside the list of lists
    for i in range(len(fileNames)):
        dummyData.append(np.genfromtxt(fileNames[i], delimiter= ',', skip_header= 1, usecols=(0)))
        minLen = min(minLen, len(dummyData[i]))
    
    tempsAdc = np.empty((len(dummyData), minLen))
    tempsCels = np.empty((len(dummyData), minLen))
    for i in range(len(dummyData)):
        tempsAdc[i] = dummyData[i][:minLen]
    tempsCels = convertAdcToCels(tempsAdc)
    
    dataTime = 0.1 * np.arange(0, minLen)   #Data was read by gui at 10Hz, ie a period of 0.1 sec
    
    targetTemp = np.array(np.genfromtxt(fileNames[0], delimiter= ',', skip_header= 1, usecols=(4)))
    targetTemp = convertAdcToCels(targetTemp[:minLen])
    
    
    
######################## Grid-based graph #########################
    figure, axis = plt.subplots(3, 2, layout="constrained")
    titles = [["Kp=20", "Kp=100"], ["Kp=200", "Kp=500"], ["Kp=1000", "Kp=2000"]]
    xlim = [dataTime[0], dataTime[-1]]
    ylim = [20, 25]
    k = 0
    for i in range(3):
        for j in range(2):   
            axis[i, j].plot(dataTime,  tempsCels[k])
            axis[i, j].plot(dataTime,  targetTemp)
            axis[i, j].set_title(titles[i][j])
            axis[i, j].set_xlim(xlim)
            axis[i, j].set_ylim(ylim)
            axis[i, j].set_xlabel("Time (s)")
            axis[i, j].set_ylabel("Temp (°C)")
            k+=1
        
    text = ("Discussion")

    figure.suptitle("Title")
    figure.text(.5, -.1, text, ha='center')
    plt.show()
    
######################## Multi-plot single graph #########################

    figure = plt.figure()
    labels = ["Kp=20", "Kp=100", "Kp=200", "Kp=500", "Kp=1000", "Kp=2000"]
    plt.title("PID Controller Kp Tuning")
    for i in range(len(tempsCels)):
        plt.plot(dataTime, tempsCels[i], label = labels[i])
    plt.plot(dataTime, targetTemp, label = "Set Point", color='b', linestyle ='dashed')
    plt.xlabel('Time (s)')
    plt.ylabel('Temperature (°C)')
    plt.xlim(dataTime[0], dataTime[-1])
    plt.ylim(20, 25)
    plt.legend(bbox_to_anchor=(1.04, 1), borderaxespad=0)
    plt.show()

if noiseCharacteristics:
   
    # open the file in read mode
    fileName1 = 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Peltier_PController_AdcNoiseCharacteristic_AveragingSample1.csv'
    fileName3 = 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Peltier_PController_AdcNoiseCharacteristic_AveragingSample3.csv'
    fileName6 = 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Peltier_PController_AdcNoiseCharacteristic_AveragingSample6.csv'
    fileName9 = 'C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Peltier_PController_AdcNoiseCharacteristic_AveragingSample9.csv'
    fileNoiseData1 = open(fileName1, 'r')
    fileNoiseData3 = open(fileName3, 'r')
    fileNoiseData6 = open(fileName6, 'r')
    fileNoiseData9 = open(fileName9, 'r')
     
    # creating dictreader object
    csvNoiseData1 = csv.DictReader(fileNoiseData1)
    csvNoiseData3 = csv.DictReader(fileNoiseData3)
    csvNoiseData6 = csv.DictReader(fileNoiseData6)
    csvNoiseData9 = csv.DictReader(fileNoiseData9)
     
    temps1 = np.genfromtxt(fileName1, delimiter= ',', skip_header= 1, usecols=(0))
    temps3 = np.genfromtxt(fileName3, delimiter= ',', skip_header= 1, usecols=(0))
    temps6 = np.genfromtxt(fileName6, delimiter= ',', skip_header= 1, usecols=(0))
    temps9 = np.genfromtxt(fileName9, delimiter= ',', skip_header= 1, usecols=(0))
    
    tempsAvg1 = np.average(temps1) * np.ones_like(temps3)   #temps3 is not a typo. Want average line to cut all the way accross the graph
    tempsAvg3 = np.average(temps3) * np.ones_like(temps3)
    tempsAvg6 = np.average(temps6) * np.ones_like(temps6)
    tempsAvg9 = np.average(temps9) * np.ones_like(temps9)
    
    tempsCels1 = (1. / (1. / 3650 * -np.log(4095.00000001 / temps1 - 1) + 1. / 298)) - 273
    tempsCels3 = (1. / (1. / 3650 * -np.log(4095.00000001 / temps3 - 1) + 1. / 298)) - 273
    tempsCels6 = (1. / (1. / 3650 * -np.log(4095.00000001 / temps6 - 1) + 1. / 298)) - 273
    tempsCels9 = (1. / (1. / 3650 * -np.log(4095.00000001 / temps9 - 1) + 1. / 298)) - 273
    
    tempsCelsAvg1 = np.average(tempsCels1) * np.ones_like(tempsCels3)   #tempsCels3 is not a typo. Want average line to cut all the way accross the graph
    tempsCelsAvg3 = np.average(tempsCels3) * np.ones_like(tempsCels3)   
    tempsCelsAvg6 = np.average(tempsCels6) * np.ones_like(tempsCels6)   
    tempsCelsAvg9 = np.average(tempsCels9) * np.ones_like(tempsCels9)   
    
    noise1 = statistics.stdev(tempsCels1)
    noise3 = statistics.stdev(tempsCels3)
    noise6 = statistics.stdev(tempsCels6)
    noise9 = statistics.stdev(tempsCels9)
 
    noiseExp3 = noise1 / np.sqrt(3)
    noiseExp6 = noise1 / np.sqrt(6)
    noiseExp9 = noise1 / np.sqrt(9)

    figure, axis = plt.subplots(2, 2, layout="constrained")
    xlim = [0, 200]
    ylim = [2185, 2197]
    ylimCels = [21.5, 21.75]
    
    axis[0, 0].plot(np.arange(0, len(tempsCels1)),  tempsCels1)
    axis[0, 0].plot(np.arange(0, len(tempsCels3)),  tempsCelsAvg1)
    axis[0, 0].set_title("n=1, SD="+"{:.3f}".format(noise1))
    axis[0, 0].set_xlim(xlim)
    axis[0, 0].set_ylim(ylimCels)
    axis[0, 0].set_xlabel("Time (GUI Ticks)")
    axis[0, 0].set_ylabel("Temp (°C)")
    
    temps3Offest = 200
    axis[0, 1].plot(np.arange(0, len(tempsCels3)-temps3Offest),  tempsCels3[temps3Offest:])
    axis[0, 1].plot(np.arange(0, len(tempsCels3)-temps3Offest),  tempsCelsAvg3[temps3Offest:])
    axis[0, 1].set_title("n=3, SD="+"{:.3f}".format(noise3))
    axis[0, 1].set_xlim(xlim)
    axis[0, 1].set_ylim(ylimCels)
    axis[0, 0].set_xlabel("Time (GUI Ticks)")
    axis[0, 0].set_ylabel("Temp (°C)")
    
    axis[1, 0].plot(np.arange(0, len(tempsCels6)),  tempsCels6)
    axis[1, 0].plot(np.arange(0, len(tempsCels6)),  tempsCelsAvg6)
    axis[1, 0].set_title("n=6, SD="+"{:.3f}".format(noise6))
    axis[1, 0].set_xlim(xlim)
    axis[1, 0].set_ylim(ylimCels)
    axis[0, 0].set_xlabel("Time (GUI Ticks)")
    axis[0, 0].set_ylabel("Temp (°C)")
    
    axis[1, 1].plot(np.arange(0, len(tempsCels9)),  tempsCels9)
    axis[1, 1].plot(np.arange(0, len(tempsCels9)),  tempsCelsAvg9)
    axis[1, 1].set_title("n=9, SD="+"{:.3f}".format(noise9))
    axis[1, 1].set_xlim(xlim)
    axis[1, 1].set_ylim(ylimCels)
    axis[0, 0].set_xlabel("Time (GUI Ticks)")
    axis[0, 0].set_ylabel("Temp (°C)")
   
    text = ("Because increased samples improves noise by a factor of √(n), we should  expect \n that the standard deviation for n = 3, 6, 9 would be "
            +"{:.3f}".format(noiseExp3) + ',' +"{:.3f}".format(noiseExp6) + ',' +"{:.3f}".format(noiseExp9) + '. '
            "This assumption \n only holds assuming the noise is Gaussian, but the results seem to vindicate the assumption.")

    figure.suptitle('Temperature Noise Characteristic')
    figure.text(.5, -.1, text, ha='center')
    plt.show()
    
    
    
    ###One basic way of reading csv
    #sampleData1 = open("C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Peltier_PController_AdcNoiseCharacteristic_AveragingSample1.csv")
    #reader1 = csv.reader(sampleData1)
    
    with open('C:/Users/chris/GitRepositories/STM32-PidControlledPeltier/Data/Peltier_PController_AdcNoiseCharacteristic_AveragingSample1.csv') as csvNoiseData1:
        readerNoiseData1 = csv.DictReader(csvNoiseData1)
        rows1 = list(readerNoiseData1)
        reverseMode1 = []
        for col in readerNoiseData1:
            reverseMode1.append([col['RegPeltierTempColdsideAdc']])
            print(col)
        print(reverseMode1)
        
    
    

##########################################################################
###   Close the serial connection
###   
##########################################################################

ser.close()
    



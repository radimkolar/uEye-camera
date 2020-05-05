#===========================================================================#
# Modified by Radim Kolar, Brno University of Technology, 2020              #
# Based on iDS Imaging, Python Examples                                     #
#===========================================================================#
#  Copyright (C) 2006 - 2018                                                #
#  IDS Imaging Development Systems GmbH                                     #
#  Dimbacher Str. 6-8                                                       #
#  D-74182 Obersulm, Germany                                                #
#                                                                           #
#  The information in this document is subject to change without notice     #
#  and should not be construed as a commitment by IDS Imaging Development   #
#  Systems GmbH. IDS Imaging Development Systems GmbH does not assume any   #
#  responsibility for any errors that may appear in this document.          #
#                                                                           #
#  This document, or source code, is provided solely as an example          #
#  of how to utilize IDS software libraries in a sample application.        #
#  IDS Imaging Development Systems GmbH does not assume any responsibility  #
#  for the use or reliability of any portion of this document or the        #
#  described software.                                                      #
#                                                                           #
#  General permission to copy or modify, but not for profit, is hereby      #
#  granted, provided that the above copyright notice is included and        #
#  reference made to the fact that reproduction privileges were granted     #
#  by IDS Imaging Development Systems GmbH.                                 #
#                                                                           #
#  IDS Imaging Development Systems GmbH cannot assume any responsibility    #
#  for the use or misuse of any portion of this software for other than     #
#  its intended diagnostic purpose in calibrating and testing IDS           #
#  manufactured cameras and software.                                       #
#                                                                           #
#===========================================================================#

# Developer Note: I tried to let it as simple as possible.
# Therefore there are no functions asking for the newest driver software or freeing memory beforehand, etc.
# The sole purpose of this program is to show one of the simplest ways to interact with an IDS camera via the uEye API.
# (XS cameras are not supported)
#---------------------------------------------------------------------------------------------------------------------------------------

#Libraries
from pyueye import ueye
from math import log
import numpy as np
import cv2
#from matplotlib import pyplot as plt
import os
#import keyboard
#import matplotlib.cm as cm
#import matplotlib.pyplot as plt
from ctypes import *
#import sys

#---------------------------------------------------------------------------------------------------------------------------------------

#Variables
hCam = ueye.HIDS(0)             #0: first available camera;  1-254: The camera with the specified camera ID
sInfo = ueye.SENSORINFO()
cInfo = ueye.CAMINFO()
pcImageMemory = ueye.c_mem_p()
MemID = ueye.int()
rectAOI = ueye.IS_RECT()
pitch = ueye.INT()
nBitsPerPixel = ueye.INT(8)    #24: bits per pixel for color mode; take 8 bits per pixel for monochrome
channels = 3                    #3: channels for color mode(RGB); take 1 channel for monochrome
m_nColorMode = ueye.INT()		# Y8/RGB16/RGB24/REG32
bytes_per_pixel = int(nBitsPerPixel / 8)
OldFrameRate = c_double() #ueye.DOUBLE()#float(0)
myFrameRate = c_double(5)
myExposure = c_double(50)
myGain = c_int(50);
maxi = float(0)
mini = float(100)
refPoint = []
speklen = 150
ii = 1
#---------------------------------------------------------------------------------------------------------------------------------------
print("START")
print()

#%%
# Starts the driver and establishes the connection to the camera
nRet = ueye.is_InitCamera(hCam, None)
if nRet != ueye.IS_SUCCESS:
    print("is_InitCamera ERROR")

# Reads out the data hard-coded in the non-volatile camera memory and writes it to the data structure that cInfo points to
nRet = ueye.is_GetCameraInfo(hCam, cInfo)
if nRet != ueye.IS_SUCCESS:
    print("is_GetCameraInfo ERROR")

# You can query additional information about the sensor type used in the camera
nRet = ueye.is_GetSensorInfo(hCam, sInfo)
if nRet != ueye.IS_SUCCESS:
    print("is_GetSensorInfo ERROR")

nRet = ueye.is_ResetToDefault( hCam)
if nRet != ueye.IS_SUCCESS:
    print("is_ResetToDefault ERROR")

# Set display mode to DIB
nRet = ueye.is_SetDisplayMode(hCam, ueye.IS_SET_DM_DIB)

# Set the right color mode
if int.from_bytes(sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_BAYER:
    # setup the color depth to the current windows setting
    ueye.is_GetColorDepth(hCam, nBitsPerPixel, m_nColorMode)
    bytes_per_pixel = int(nBitsPerPixel / 8)
    print("IS_COLORMODE_BAYER: ", )
    print("\tm_nColorMode: \t\t", m_nColorMode)
    print("\tnBitsPerPixel: \t\t", nBitsPerPixel)
    print("\tbytes_per_pixel: \t\t", bytes_per_pixel)
    print()

elif int.from_bytes(sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_CBYCRY:
    # for color camera models use RGB32 mode
    m_nColorMode = ueye.IS_CM_BGRA8_PACKED
    nBitsPerPixel = ueye.INT(32)
    bytes_per_pixel = int(nBitsPerPixel / 8)
    print("IS_COLORMODE_CBYCRY: ", )
    print("\tm_nColorMode: \t\t", m_nColorMode)
    print("\tnBitsPerPixel: \t\t", nBitsPerPixel)
    print("\tbytes_per_pixel: \t\t", bytes_per_pixel)
    print()

elif int.from_bytes(sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_MONOCHROME:
    # for color camera models use RGB32 mode
    m_nColorMode = ueye.IS_CM_MONO8
    nBitsPerPixel = ueye.INT(8)
    bytes_per_pixel = int(nBitsPerPixel / 8)
    print("IS_COLORMODE_MONOCHROME: ", )
    print("\tm_nColorMode: \t\t", m_nColorMode)
    print("\tnBitsPerPixel: \t\t", nBitsPerPixel)
    print("\tbytes_per_pixel: \t\t", bytes_per_pixel)
    print()

else:
    # for monochrome camera models use Y8 mode
    m_nColorMode = ueye.IS_CM_MONO8
    nBitsPerPixel = ueye.INT(8)
    bytes_per_pixel = int(nBitsPerPixel / 8)
    print("else")

# Can be used to set the size and position of an "area of interest"(AOI) within an image
nRet = ueye.is_AOI(hCam, ueye.IS_AOI_IMAGE_GET_AOI, rectAOI, ueye.sizeof(rectAOI))
if nRet != ueye.IS_SUCCESS:
    print("is_AOI ERROR")

width = rectAOI.s32Width
height = rectAOI.s32Height
width2 = round(width/2)
height2 = round(height/2)

# Prints out some information about the camera and the sensor
print("Camera model:\t\t", sInfo.strSensorName.decode('utf-8'))
print("Camera serial no.:\t", cInfo.SerNo.decode('utf-8'))
print("Maximum image width:\t", width)
print("Maximum image height:\t", height)
print("Maximum 1/2 image width:\t", round(float(width.value)/2))
#print("Maximum image width:\t", float(width/2)
print()

#---------------------------------------------------------------------------------------------------------------------------------------

# Allocates an image memory for an image having its dimensions defined by width and height and its color depth defined by nBitsPerPixel
nRet = ueye.is_AllocImageMem(hCam, width, height, nBitsPerPixel, pcImageMemory, MemID)
if nRet != ueye.IS_SUCCESS:
    print("is_AllocImageMem ERROR")
else:
    print("Mem Allocation OK")
    # Makes the specified image memory the active memory
    nRet = ueye.is_SetImageMem(hCam, pcImageMemory, MemID)
    if nRet != ueye.IS_SUCCESS:
        print("is_SetImageMem ERROR")
    else:
        # Set the desired color mode
        nRet = ueye.is_SetColorMode(hCam, m_nColorMode)
        

nRet = ueye.is_GetFramesPerSecond( hCam, OldFrameRate )
if nRet != ueye.IS_SUCCESS:
    print("GetFrame ERROR")
else:
    print("FramesPerSecond:\t", OldFrameRate.value)

# Set FRAME RATE    
nRet = ueye.is_SetFrameRate( hCam, myFrameRate, c_double() )
if nRet != ueye.IS_SUCCESS:
    print("SetFrame ERROR:\t", nRet)
else:
    print("FramesPerSecond:\t", myFrameRate)

# Set EXPOSURE TIME
 # nRet = is_Exposure(m_hCam, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&m_ExposureTime, sizeof(m_ExposureTime));
#IS_EXPOSURE_CMD_SET_EXPOSURE = 12
nRet = ueye.is_Exposure( hCam, 12, myExposure, 8) #sizeof(myExposure) )
if nRet != ueye.IS_SUCCESS:
    print("Exposure ERROR:\t", nRet)
else:
    print("Exposure set to:\t", myExposure)

# Set GAIN    
nRet = ueye.is_SetHardwareGain( hCam, myGain, -1, -1, -1 )
if nRet != ueye.IS_SUCCESS:
    print("SetGain ERROR:\t", nRet)
else:
    print("Gain set to:\t", myGain)


# Activates the camera's live video mode (free run mode)
nRet = ueye.is_CaptureVideo(hCam, ueye.IS_DONT_WAIT)
if nRet != ueye.IS_SUCCESS:
    print("is_CaptureVideo ERROR")

# Enables the queue mode for existing image memory sequences
nRet = ueye.is_InquireImageMem(hCam, pcImageMemory, MemID, width, height, nBitsPerPixel, pitch)
if nRet != ueye.IS_SUCCESS:
    print("is_InquireImageMem ERROR")
else:
    print("Press q to leave the programm")

#---------------------------------------------------------------------------------------------------------------------------------------
def myclick(event, x, y, flags, param):
    global refPoint
    if event == cv2.EVENT_LBUTTONDOWN:
        refPoint = [x, y]
#------------------------------------------------
cv2.namedWindow("Spectrum")
cv2.setMouseCallback("Spectrum", myclick) 

while(nRet == ueye.IS_SUCCESS):
    # In order to display the image in an OpenCV window we need to...
    # ...extract the data of our image memory
    array = ueye.get_data(pcImageMemory, width, height, nBitsPerPixel, pitch, copy=False)

    # bytes_per_pixel = int(nBitsPerPixel / 8)
    
    # ...reshape it in an numpy array...
    framergb = np.reshape(array,(height.value, width.value, bytes_per_pixel))
    frame = cv2.cvtColor(framergb, cv2.COLOR_BGR2GRAY)

    # ...resize the image by a half
    frame2 = cv2.resize(frame,(0,0),fx=0.5, fy=0.5)
    
#---------------------------------------------------------------------------------------------------------------------------------------
    # Compute spectra, fftshift, divide by image size
    Spek = np.fft.fft2(frame2)
    Spek2 = np.fft.fftshift(Spek)/(width*height)

    # magnitude part of the spectra, alternatively log
    magnitude_spectrum = np.abs(Spek2)
    #magnitude_spectrum = abs(np.log(magnitude_spectrum+1))
    
    # spectrum normalization
    mini, maxi, _, _ = cv2.minMaxLoc(magnitude_spectrum)
    magnitude_spectrum = 255*(magnitude_spectrum-mini)/(maxi-mini)
    
    # frame normalization
    mini, maxi, _, _ = cv2.minMaxLoc(frame2)
    frame2 = (frame2-mini)/(maxi-mini)
        
    # show in OpenCV windows
    cv2.imshow("Live image", frame2)    
    cv2.imshow("Spectrum", magnitude_spectrum)
    
    # Press q if you want to end the loop; press s if you want to save image
    # Press u/d is you want to increase/decrease spectral window size 
    mykey = cv2.waitKey(3)
    if  mykey & 0xFF == ord('s'):
        directory = r'C:\Users\kolarr\Desktop\OffAxis'
        os.chdir(directory) 
        filename = 'savedImage' + str(ii) + '.jpg'        
        cv2.imwrite(filename, frame)
        ii = ii + 1        
    elif mykey & 0xFF == ord('u'):
        speklen = speklen + 5
        print(speklen)
    elif mykey & 0xFF == ord('d'):
        speklen = speklen - 5
        print(speklen)
    elif mykey & 0xFF == ord('q'):
        break

    # if click in spectral window, make the processing and imaging
    if len(refPoint)>0:
        SpekPart = Spek2[refPoint[1]-speklen:refPoint[1]+speklen, refPoint[0]-speklen:refPoint[0]+speklen ]        
        SpekPart = np.fft.ifftshift(SpekPart)*(speklen*speklen)        
        imagespek = np.fft.ifft2(SpekPart )
        imagespek = np.abs(imagespek)
        imagespek = cv2.resize( imagespek,(0,0),fx=2.0, fy=2.0)
    
        mini, maxi, _, _ = cv2.minMaxLoc(imagespek)
        imagespek= (imagespek-mini)/(maxi-mini)
        cv2.imshow("Image from spectra", imagespek)
        
#---------------------------------------------------------------------------------------------------------------------------------------

# Releases an image memory that was allocated using is_AllocImageMem() and removes it from the driver management
ueye.is_FreeImageMem(hCam, pcImageMemory, MemID)

# Disables the hCam camera handle and releases the data structures and memory areas taken up by the uEye camera
ueye.is_ExitCamera(hCam)

# Destroys the OpenCv windows
cv2.destroyAllWindows()

print()
print("END")

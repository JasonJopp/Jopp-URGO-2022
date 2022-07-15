# -----------------------------------------------------------------------------
# Code Purpose: Given a drive command, move a Sphero Rover using the command.
# Returns flag based on if the RVRs color sensor detects a certain color.
#
# NOTE: Right now driver() uses the same speed for both left & right tracks,
# this is not required. Left/right speed params can be set separately to 
# implement gradual turning while moving forwards/backwards.
#
# Author(s): Jason Jopp, <your name here>
# -----------------------------------------------------------------------------

import asyncio
import numpy as np
import os
import sys
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../sphero-sdk-raspberrypi-python')))
from sphero_sdk import RawMotorModesEnum, Colors, SpheroRvrObserver

# This flag triggers if the RVR drives over a specific color
colorFlag = False


def color_detected_handler(color_detected_data):
    """
    Gets color information from RVRs color scanner, 
    returns flag depending on if the color matches the set parameters.
    """
    
    global colorFlag

    # Sets detected RGB values from RVR color sensor to the rgb list variable
    rgb = [
        color_detected_data['ColorDetection']['R'],
        color_detected_data['ColorDetection']['G'],
        color_detected_data['ColorDetection']['B']
        ]

    # Compares sensed color to params, if it matches, sets flag to true
    # This currently checks for color: Red
    if (rgb[0] > 220) and (rgb[1] < 60) and (rgb[2] < 60):
        colorFlag = True

async def driver(rvr, leftMode, rightMode, driveTime = 2, 
    speed = 64, cmdName = "Unnamed Command", colorCode = [255,255,255]):
    """
    Drives a given RVR in a given direction.
    Params: rover obj, left track drive mode (0,1,2), 
    right track drive mode (0,1,2), drive time (seconds), speed,
    command name, and led color code (RGB) to use when driving.
    NOTE: Right/Left track modes: 0 = stop, 1 = forward, 2 = reverse
    """

    # Imports global values
    global colorFlag

    # Resets colorFlag to default False value before RVR drives
    colorFlag = False

    # This is the sample speed (ms) of the RVRs bottom facing color sensor
    colorSampleSpeed = 50

    # Various checks on drive variables, must fall within these bounds to work
    if not (0 <= speed <= 255):
        print("ERROR: Invalid speed entered, must be between 0 and 255.")
        exit()

    if not (driveTime > 0):
        print("ERROR: Invalid time entered, must be greater than zero.")
        exit()

    if not (0 <= leftMode <= 2) and isinstance(leftMode, int):
        print("ERROR: lMode was not an int between 0-2, incl.")
        exit()
    
    if not (0 <= rightMode <= 2) and isinstance(rightMode, int):
        print("ERROR: lMode was not an int between 0-2, incl.")
        exit()


    # Divides driveTime between amountTimes and finalTime, rover acts in two
    # second increments for UNK reason, this way the last movement can be
    # cut off at the 'finalTime' amount, with 'amountTimes' being how many 2
    # second rotations are required before that to meet the required driveTime.
    # This causes rover pause between commands, but without it the rover always
    # moves in increments of two seconds when using the raw motor input mode.
    amountTimes = int(np.floor(driveTime/2))
    finalTime = round(driveTime%2, 3)

    # Wakes the RVR from sleep mode
    rvr.wake()

    # Gives RVR time to wake up, RVR sometimes misses commands without this.
    await asyncio.sleep(1)
    
    # Prevents old drive directions from interfering with new commands
    rvr.reset_yaw()
    
    # Sets RVR leds to white when moving, unless other color code given
    rvr.led_control.set_all_leds_rgb(*colorCode)

    # Turns on the RVRs bottom facing color sensor
    rvr.sensor_control.start(interval=colorSampleSpeed)

    # Runs the rover amount of two second increments to meet driveTime amount
    while amountTimes > 0:   
        rvr.raw_motors(
            left_mode=leftMode,
            left_duty_cycle=speed,
            right_mode=rightMode,
            right_duty_cycle=speed
        )
        await asyncio.sleep(2)
        amountTimes -= 1
    
    # Runs sub-two-second movements if remaining driveTime != 2 seconds
    if finalTime > 0:
        rvr.raw_motors(
            left_mode=leftMode,
            left_duty_cycle=speed,
            right_mode=rightMode,
            right_duty_cycle=speed
        )
        # Stops above movement depending on finaltime variable below.
        await asyncio.sleep(finalTime)
        rvr.raw_motors(
            left_mode=0,
            left_duty_cycle=0,
            right_mode=0,
            right_duty_cycle=0
        )
    
    # Turns off the RVRs bottom facing color sensor
    rvr.sensor_control.stop()

    # Gives RVR time to stop before taking a photo, otherwise image is blurry
    time.sleep(.25)
    
    # Sets rover leds to white, default waiting state
    rvr.led_control.set_all_leds_rgb(red=255, green=255, blue=255)

    return colorFlag

rvr = SpheroRvrObserver()
for i in range(5):
    asyncio.run(driver(rvr, 1, 2, .15, 180))
    time.sleep(.25)

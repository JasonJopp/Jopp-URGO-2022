# -----------------------------------------------------------------------------
# Code Purpose: Given a drive command, move a Sphero Rover using the command.
#
# NOTE: Right now driver() uses the same speed for both left & right tracks,
# this is not a requirement. So if you want you can add left/right speed params
# and trying to implement gradual turning while moving forward/back.
#
# Author(s): Jason Jopp, <your name here>
# -----------------------------------------------------------------------------

import asyncio, numpy as np, time
from sphero_sdk import RawMotorModesEnum, Colors

async def driver(rvr, leftMode, rightMode, driveTime = 2, 
    speed = 64, cmdName = "Unnamed Command", colorCode = [255,255,255]):
    """
    Drives a given RVR in a given direction.
    Params: rover obj, left track drive mode (0,1,2), 
    right track drive mode (0,1,2), drive time (seconds), speed,
    command name, and led color code (RGB) to use when driving.
    NOTE: Right/Left track modes: 0 = stop, 1 = forward, 2 = reverse
    """

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
    # Gives RVR time to stop before taking a photo, otherwise image is blurry
    time.sleep(.15)
    # Sets rover leds to white, default waiting state
    rvr.led_control.set_all_leds_rgb(red=255, green=255, blue=255)

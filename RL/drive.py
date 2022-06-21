import asyncio, numpy as np, time
from sphero_sdk import RawMotorModesEnum, Colors

async def driver(rvr, leftMode, rightMode, driveTime = 2, 
    speed = 64, cmdName = "Unnamed Command", colorCode = [0,255,0]) -> None:
    """
    Drives a given RVR in a given direction.
    Params: rover obj, left track drive mode (0,1,2), 
    right track drive mode (0,1,2), drive time (seconds), speed.
    NOTE: Right/Left modes: 0 = stop, 1 = forward, 2 = reverse
    """

    # Various checks on drive variables
    if not (0 <= speed <= 255):
        print("DRIVE VARROR: Invalid speed entered, must be between 0 and 255.")
        exit()

    if not (driveTime > 0): # It is difficult to move a rover for negative secs.
        print("DRIVE VARROR: Invalid time entered, must be greater than zero.")
        exit()

    if not (0 <= leftMode <= 2) and isinstance(leftMode, int):
        print("DRIVE VARROR: lMode was not an int between 0-2, incl.")
        exit()
    
    if not (0 <= rightMode <= 2) and isinstance(rightMode, int):
        print("DRIVE VARROR: lMode was not an int between 0-2, incl.")
        exit()

    # Used to drive the rover forward a certain amount of time, rover acts in
    # two second increments for UNK reason, this way the last movement can be
    # cut off at the 'finalTime' amount, with 'amountTimes' being how many 2
    # second rotations are required before that.

    # NOTE: This may make rover pause between commands, but I don't know how to
    # interrupt the RVR's movement without a stop command, which lasts two seconds.
    amountTimes = int(np.floor(driveTime/2))
    finalTime = round(driveTime%2, 3)

    rvr.wake()

    # Gives the RVR time to wake up. Inconsistent without wake period.
    await asyncio.sleep(1)
    
    rvr.reset_yaw()
    
    # Sets RVR leds to green when moving
    rvr.led_control.set_all_leds_rgb(*colorCode)
    while amountTimes > 0:   
        rvr.raw_motors(
            left_mode=leftMode,
            left_duty_cycle=speed,
            right_mode=rightMode,
            right_duty_cycle=speed
        )
        await asyncio.sleep(2)
        amountTimes -= 1
    
    # Performs additional movement if movetime wasn't divisible by two.
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
    rvr.led_control.set_all_leds_rgb(red=255, green=165, blue=0)
    time.sleep(.15)
    #time.sleep(4)
    rvr.led_control.set_all_leds_rgb(red=255, green=255, blue=255)

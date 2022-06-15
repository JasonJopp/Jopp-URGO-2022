import asyncio, time, numpy as np
from sphero_sdk import RawMotorModesEnum

async def driver(rvr, direction, driveTime = 2, speed = 64,):
    """
    Drives a given RVR in a given direction.
    Default speed is 64.
    """
    if not (0 <= speed <= 255):
        print("ERROR: Invalid speed entered, must be between 0 and 255.")
        exit()

    if not (driveTime > 0):
        print("ERROR: Invalid time entered, must be greater than zero.")
        exit()

    # Used to drive the rover forward a certain amount of time, because the rover
    # acts in two second increments by default, this way the last movement can
    # be cut off at the 'finalTime' amount, with 'amountTimes' being how many 2
    # second rotations are required before that.
    amountTimes = int(np.floor(driveTime/2))
    finalTime = round(driveTime%2, 3)

    await rvr.wake()
    # The rover moves in two second intervals by default. So first the RVR
    # loops in two second intervals until the last sub-two second movement interval
    # where the movement is interrupted in the final loop by a stop command.
    # NOTE: This may make the rovers movement not smooth, but don't know how to
    # interrupt the RVR's movement without a stop command, which lasts two seconds.
    # NOTE: Right/Left modes for raw motors: 0 = stop, 1 = forward, 2 = reverse

    # Uncomment to give RVR time to wake up, not sure if useful in non-sleep mode
    await asyncio.sleep(1)
    
    await rvr.reset_yaw()
    while amountTimes > 0:
        if (direction == "forward"):    
            await rvr.raw_motors(
                left_mode=1,
                left_duty_cycle=speed,  # Valid duty cycle range is 0-255
                right_mode=1,
                right_duty_cycle=speed  # Valid duty cycle range is 0-255
            )
            
        elif (direction == "reverse"):
            await rvr.raw_motors(
                left_mode=2,
                left_duty_cycle=speed,  # Valid duty cycle range is 0-255
                right_mode=2,
                right_duty_cycle=speed  # Valid duty cycle range is 0-255
            )
        
        elif (direction == "left"):
            await rvr.raw_motors(
                left_mode=2,
                left_duty_cycle=speed,  # Valid duty cycle range is 0-255
                right_mode=1,
                right_duty_cycle=speed  # Valid duty cycle range is 0-255
            )
        
        elif (direction == "right"):
            await rvr.raw_motors(
                left_mode=1,
                left_duty_cycle=speed,  # Valid duty cycle range is 0-255
                right_mode=2,
                right_duty_cycle=speed  # Valid duty cycle range is 0-255
            )
        
        elif (direction == "stop"):
            await rvr.raw_motors(
                left_mode=0,
                left_duty_cycle=0, # Valid duty cycle range is 0-255
                right_mode=0,
                right_duty_cycle=0 # Valid duty cycle range is 0-255
            )
        await asyncio.sleep(2)
        amountTimes -= 1
    
    # Performs additional movement if movetime wasn't divisible by two.
    if finalTime > 0:
        if (direction == "forward"):    
            await rvr.raw_motors(
                left_mode=1,
                left_duty_cycle=speed,  # Valid duty cycle range is 0-255
                right_mode=1,
                right_duty_cycle=speed  # Valid duty cycle range is 0-255
            )
            await asyncio.sleep(finalTime)
            await rvr.raw_motors(
                left_mode=0,
                left_duty_cycle=0, # Valid duty cycle range is 0-255
                right_mode=0,
                right_duty_cycle=0# Valid duty cycle range is 0-255
            )
            
        elif (direction == "reverse"):
            await rvr.raw_motors(
                left_mode=RawMotorModesEnum.reverse.value,
                left_duty_cycle=speed,  # Valid duty cycle range is 0-255
                right_mode=RawMotorModesEnum.reverse.value,
                right_duty_cycle=speed # Valid duty cycle range is 0-255
            )
            await asyncio.sleep(finalTime)
            await rvr.raw_motors(
                left_mode=RawMotorModesEnum.off.value,
                left_duty_cycle=0, # Valid duty cycle range is 0-255
                right_mode=RawMotorModesEnum.off.value,
                right_duty_cycle=0# Valid duty cycle range is 0-255
            )
        
        elif (direction == "left"):
            await rvr.raw_motors(
                left_mode=RawMotorModesEnum.reverse.value,
                left_duty_cycle=speed,  # Valid duty cycle range is 0-255
                right_mode=RawMotorModesEnum.forward.value,
                right_duty_cycle=speed  # Valid duty cycle range is 0-255
            )
            await asyncio.sleep(finalTime)
            await rvr.raw_motors(
                left_mode=RawMotorModesEnum.off.value,
                left_duty_cycle=0, # Valid duty cycle range is 0-255
                right_mode=RawMotorModesEnum.off.value,
                right_duty_cycle=0# Valid duty cycle range is 0-255
            )
        
        elif (direction == "right"):
            await rvr.raw_motors(
                left_mode=RawMotorModesEnum.forward.value,
                left_duty_cycle=speed,  # Valid duty cycle range is 0-255
                right_mode=RawMotorModesEnum.reverse.value,
                right_duty_cycle=speed  # Valid duty cycle range is 0-255
            )
            await asyncio.sleep(finalTime)
            await rvr.raw_motors(
                left_mode=RawMotorModesEnum.off.value,
                left_duty_cycle=0, # Valid duty cycle range is 0-255
                right_mode=RawMotorModesEnum.off.value,
                right_duty_cycle=0# Valid duty cycle range is 0-255
            )
        
        elif (direction == "stop"):
            await rvr.raw_motors(
                left_mode=RawMotorModesEnum.off.value,
                left_duty_cycle=0, # Valid duty cycle range is 0-255
                right_mode=RawMotorModesEnum.off.value,
                right_duty_cycle=0# Valid duty cycle range is 0-255
            )
            await asyncio.sleep(finalTime)
            await rvr.raw_motors(
                left_mode=RawMotorModesEnum.off.value,
                left_duty_cycle=0, # Valid duty cycle range is 0-255
                right_mode=RawMotorModesEnum.off.value,
                right_duty_cycle=0# Valid duty cycle range is 0-255
            )
    # Delay to allow RVR to spin motors
    await asyncio.sleep(1)

    #await rvr.close()

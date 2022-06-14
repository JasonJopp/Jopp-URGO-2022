import asyncio
import time
from sphero_sdk import RawMotorModesEnum

async def driver(rvr, direction, speed = 64):
    """
    Drives a given RVR in a given direction.
    Default speed is 64.
    """
    if not (0 <= speed <= 255):
        print("Invalid speed entered, must be between 0 and 255.")
        exit()

    await rvr.wake()
    # NOTE: Make it so the program can run for more than two seconds,
    # This can be done by finding how many 2 second loops are required
    # Running those, and finding the remaining time and running that at
    # the end using asyncio.sleep(DOUBLE), then a drive stop command after

    # Uncomment to give RVR time to wake up, not sure if useful in non-sleep mode
    await asyncio.sleep(1)

    await rvr.reset_yaw()

    if (direction == "forward"):
        await rvr.raw_motors(
            left_mode=RawMotorModesEnum.forward.value,
            left_duty_cycle=speed,  # Valid duty cycle range is 0-255
            right_mode=RawMotorModesEnum.forward.value,
            right_duty_cycle=speed  # Valid duty cycle range is 0-255
        )
        
    elif (direction == "reverse"):
        await rvr.raw_motors(
            left_mode=RawMotorModesEnum.reverse.value,
            left_duty_cycle=speed,  # Valid duty cycle range is 0-255
            right_mode=RawMotorModesEnum.reverse.value,
            right_duty_cycle=speed # Valid duty cycle range is 0-255
        )
    
    elif (direction == "left"):
        await rvr.raw_motors(
            left_mode=RawMotorModesEnum.reverse.value,
            left_duty_cycle=speed,  # Valid duty cycle range is 0-255
            right_mode=RawMotorModesEnum.forward.value,
            right_duty_cycle=speed  # Valid duty cycle range is 0-255
        )
    
    elif (direction == "right"):
        await rvr.raw_motors(
            left_mode=RawMotorModesEnum.forward.value,
            left_duty_cycle=speed,  # Valid duty cycle range is 0-255
            right_mode=RawMotorModesEnum.reverse.value,
            right_duty_cycle=speed  # Valid duty cycle range is 0-255
        )
    
    elif (direction == "stop"):
        await rvr.raw_motors(
            left_mode=RawMotorModesEnum.off.value,
            left_duty_cycle=0, # Valid duty cycle range is 0-255
            right_mode=RawMotorModesEnum.off.value,
            right_duty_cycle=0# Valid duty cycle range is 0-255
        )

    # Delay to allow RVR to spin motors
    await asyncio.sleep(1)

    #await rvr.close()



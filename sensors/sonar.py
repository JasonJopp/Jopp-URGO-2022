import RPi.GPIO as GPIO
import time

# Specifies which number-system is being used, in this case Broadcom-chip
GPIO.setmode(GPIO.BCM)

# Set Raspi GPIO pins to specific sonar functions
GPIO_TRIGGER = 23
GPIO_ECHO = 24

# Sets GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def sonarDistance():
    time.sleep(1) # Sets trigger to high
    GPIO.output(GPIO_TRIGGER, True)
    # Set Trigger after 0.01ms to low
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    startTime = time.time()
    stopTime = time.time()

    # Saves the start time
    while GPIO.input(GPIO_ECHO) == 0:
        startTime = time.time()
    
    # Saves time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        stopTime = time.time()
        # Breaks the loop if the sonar doesn't get an echo soon enough.
        if stopTime - startTime > .05:
            break

    # Finds time difference between start and end time to find travel time
    timeDiff = stopTime - startTime

    # Gets distance by multiplying timeDiff by the sonic speed (34300 cm/s)
    # Then divide by two, because distance is send/return distance
    distance = (timeDiff * 34300) / 2

    return distance


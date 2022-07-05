import asyncio
import cv2 as cv
import numpy as np
import os
import random
import RPi.GPIO as GPIO
import sys
import time
import warnings

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),
    '../../sphero-sdk-raspberrypi-python/')))
from sphero_sdk import SpheroRvrObserver, Colors, RvrStreamingServices
from videoGet import VideoGet

from drive import driver, color_detected_handler
from detector_class import Detector, pink_box, blue_bucket

class ServoingEnvironment:

    def __init__(self) -> None:
        # Disables warning from GPIO, common one is the GPIO is already in use
        GPIO.setwarnings(False)
        
        # Set Raspi GPIO pins to specific sonar functions
        self.GPIO_TRIGGER = 23
        self.GPIO_ECHO = 24

        # Specifies Broadcom-chip number system is being used for GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)

        # Amount of sonar readings averaged together for returned distance 
        self.sonarSampleSize = 3
        # Holds individual sonar distance samples before averaging
        self.sonarDistances = []

        # Creates RVR (Observer) object
        self.rvr = SpheroRvrObserver()
        
        # Enables the rovers bottom facing color sensor
        self.rvr.enable_color_detection(is_enabled=True)
        
        # Creates the handler for the RVR's color detection
        self.rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.color_detection,
            handler=color_detected_handler
        )

        # Creates flag for when RVR detects a specific color with it's sensor
        self.colorFlag = True

        # Lists out actions that the rover can take.
        # NOTE: [rvrObject, L-trk drive mode, R-trk drive mode,.. 
        # ..drive time (seconds), speed, command name, led color code]
        # Drive Modes: 0: Stop, 1: Forward, 2: Reverse
        self.actions = [
            [self.rvr, 1, 2, .15, 180, "Hard Right"],
            [self.rvr, 1, 2, .1, 180, "Right"],
            [self.rvr, 1, 2, .05, 180, "Soft Right"],
            [self.rvr, 1, 1, .787, 160, "80 cm Forward"],
            [self.rvr, 1, 1, .2535, 160, "20 cm Forward"],
            [self.rvr, 1, 1, .158, 160, "10 cm Forward"],
            [self.rvr, 1, 1, .102, 160, "5 cm Forward"],
            [self.rvr, 2, 1, .05, 180, "Soft Left"],
            [self.rvr, 2, 1, .1, 180, "Left"],
            [self.rvr, 2, 1, .15, 180, "Hard Left"]
        ]
        
        # Unused but valid actions:
        # [self.rvr, 2, 2, .25, 180, "Reverse"]


        # Gets number of possible actions by measuring actions list length
        self.numActions = len(self.actions)
        
        # Sets the number of vertical bins the frame is sliced into
        self.image_divisions = 7
        
        # Width of video frame, can be found using an OpenCV function
        self.image_width = 640

        # This is used to decide which sonar distance state the RVR in
        self.distanceStateDict = [[240,0], [160, 1], [80,2], [20,3], [5,4], [0,5]]
        
        # Sets number of possible states, added state is for no blob detected
        self.numStates = (self.image_divisions * len(self.distanceStateDict)) + 1

        # Signifies that blob left RVR's view, or if RVR moved out of bounds
        self.failState = self.numStates - 1

        # Gets middle slice of image divisions for reward state
        self.reward_state = (((self.image_divisions)//2) * \
        len(self.distanceStateDict)) + self.distanceStateDict[-1][1]
        
        self.current_state = self.failState

        # Creates a detector for blob detection of target object
        self.target_detector = Detector(pink_box)

        # Creates a detector for blob detection of beacon object
        self.beacon_detector = Detector(blue_bucket)

    def sonarDistance(self):
        """Returns an average sonar distance, with a set sample size."""
        time.sleep(.05)
        GPIO.output(self.GPIO_TRIGGER, True)
        # Set Trigger after 0.01ms to low
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)
        startTime = time.time()
        stopTime = time.time()

        # Saves the start time
        breakTime = time.time()
        while GPIO.input(self.GPIO_ECHO) == 0:
            startTime = time.time()
            if startTime - breakTime > .05:
                stopTime = np.nan
                break
        # Saves time of arrival
        while GPIO.input(self.GPIO_ECHO) == 1:
            stopTime = time.time()
            # Breaks the loop if the sonar doesn't get an echo soon enough.
            if stopTime - startTime > .05:
                stopTime = np.nan
                break

        # Time difference between start and end time to find travel time
        if not np.isnan(stopTime):
            timeDiff = stopTime - startTime

            # Gets distance by multiplying timeDiff by the sonic speed (34300 cm/s)
            # Then divide by two, because distance is send/return distance
            sonarDistance = (timeDiff * 34300) / 2
        else:
            sonarDistance = np.nan
        
        return sonarDistance

    def randomRestart(self):
        # Generates a random-ish drive command for restarting the test
        restartLeftTrack = random.randint(1,2)
        if restartLeftTrack == 1:
            restartRightTrack = 2
        else:
            restartRightTrack = 1
        restartDriveCommand = [self.rvr, restartLeftTrack, restartRightTrack, 
        random.uniform(.1,.5), 180, "Restart", [255,0,255]]

        return restartDriveCommand, restartLeftTrack, restartRightTrack

    def reset_with_beacon(self, videoGetter):
        self.locate_furthest_beacon(videoGetter)
        
        # visually servo to beacon
        xy, size = self.get_beacon(videoGetter)
        print(f'Ready to servo to {size}')
        while size == None:
            print('acquiring beacon for servoing')
            # it is losing the beacon
            xy, size = self.get_beacon(videoGetter)

        while size < 65:
            # determine % from center of frame (signed)
            # -1 is at left edge and +1 at right, 0 in middle
            error_from_center = xy[0] / (self.image_width/2) - 1
            err = abs(error_from_center)
            #time = .13*pow(err,3) + -.3*pow(err,2) + .33*err + -.01
            if err <= .15:    # go straight
                asyncio.run(driver(*[self.rvr, 1, 1, .4, 180, "forward beacon", [0,0,255]]))
            elif error_from_center < 0: # need to turn left
                asyncio.run(driver(*[self.rvr, 2, 1, .05, 180, "left beacon", [0,0,255]]))
            else: # need to turn right
                asyncio.run(driver(*[self.rvr, 1, 2, .05, 180, "right beacon", [0,0,255]]))
            
            xy, size = self.get_beacon(videoGetter)
            while size == None:
                xy, size = self.get_beacon(videoGetter)
                print('acquiring beacon')
            # print(f'after servo to {xy,size}')

    def locate_furthest_beacon(self,videoGetter):
        '''
        locate beacon farthest from current position. 
        rotate in a circle until both beacons are located.
        determine which is further, 
        rotating back if necessary to make sure the beacon is in frame
        '''
        # Reverse so it moves away from the box a little after finding it.
        asyncio.run(driver(*[self.rvr, 2, 2, .25, 180, "Reverse"]))

        restartDriveCommand, restartLeftTrack, restartRightTrack = self.randomRestart()
        
        # Defines the two drive commands used to acquire beacon locations
        scanDrive = [self.rvr, restartLeftTrack, restartRightTrack, .18, 180, "Scanning", [0,0,255]]
        scanDriveAlt = [self.rvr, restartRightTrack, restartLeftTrack, .18, 180, "Scanning", [0,0,255]]

        # Rotates until first beacon is found
        print('Rotating until first beacon is found')
        size_first = None
        while size_first == None:
            xy, size_first = self.get_beacon(videoGetter)
            if size_first != None:
                break
            asyncio.run(driver(*scanDrive))

        # Rotates off first beacon, so it's not counted as the 2nd beacon too 
        print(f'Rotating away from first beacon. Beacon 1 size: {size_first}')
        size = size_first
        while size != None:
            xy, size = self.get_beacon(videoGetter)
            if size == None:
                break
            asyncio.run(driver(*scanDrive))

        # Rotates until second beacon is found
        print('Rotating until 2nd beacon is found..')
        size_second = None
        while size_second == None:
            xy, size_second = self.get_beacon(videoGetter)
            if size_second != None:
                break
            asyncio.run(driver(*scanDrive))

        # compare sizes and rotate to the smallest one
        if size_first < size_second:
            print('Rotating back to smaller first beacon')
            # Rotates back towards the first beacon by swaping track modes
            asyncio.run(driver(*scanDriveAlt))
            asyncio.run(driver(*scanDriveAlt))

            # rotate until back to first beacon
            size = None
            while size == None:
                xy, size = self.get_beacon(videoGetter)
                if size != None:
                    break
                asyncio.run(driver(*scanDriveAlt))

    
    def reset(self, videoGetter):
        """This has the RVR place itself into a somewhat-random starting position by rotating"""
        print("RVR being reset")
        if self.colorFlag:
            self.reset_with_beacon(videoGetter) 
            self.colorFlag = False
        state = self.failState
        restartDriveCommand, restartLeftTrack, restartRightTrack = self.randomRestart()
        asyncio.run(driver(*restartDriveCommand))
        
        # The RVR will rotate until it sees a blob. (Search mode)
        while state == self.failState:
            state = self.get_state(videoGetter)
            if state != self.failState:
                break
            asyncio.run(driver(*[self.rvr, restartLeftTrack, restartRightTrack, .18, 180, "Scanning"]))
        return state
    
    def get_state(self, videoGetter):
        """
        Gets a video frame, searches for blobs in the frame using the detector.
        """
        # Gets frame from videoGetter obj
        frame = videoGetter.frame
        avgXY, avgSize = self.target_detector.blob_detector(frame)
        
        # Draws circle on blob location, if available
        if not np.isnan(avgXY[0]):
            frame = cv.circle(frame, (int(avgXY[0]), int(avgXY[1])), int(avgSize//2), (0,255,0), 3)
            
            # Only runs sonar if blob was found
            for i in range(self.sonarSampleSize):
                self.sonarDistances.append(self.sonarDistance())
                # Catches mean of empty slice warning
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore", category=RuntimeWarning)
                    # Averages sample sonar distances, ignoring nans, returns the average
                    self.avgSonarDistance = np.nanmean(self.sonarDistances)
                # Wipes sample sonar distances list
                self.sonarDistances.clear()
            print(f"Sonar Distance: {self.avgSonarDistance}")
        else:
            self.avgSonarDistance = np.nan
        
        # Displays the frame in a new window
        cv.imshow('Frame', frame)

        # Returns state dependent on if there is a blob on frame or not, and
        # whether or not the RVR drove over a color that is not allowed
        if self.colorFlag or (np.isnan(avgXY[0]) or np.isnan(self.avgSonarDistance)):
            state = self.failState
        else:
            # Gets the state of the x-coordinate of the blob
            blobXLocationState = int(avgXY[0]*self.image_divisions/self.image_width)

            # Checks blobSizeStateList to assign blobSizeState to a number based on the blobRatio
            for distanceBin in self.distanceStateDict:
                if self.avgSonarDistance > distanceBin[0]:
                    blobDistanceState = distanceBin[1]
                    break
            # Combines the blob x-coordinate state and blob size into a single state
            state = (blobXLocationState * len(self.distanceStateDict)) + blobDistanceState
            print(f"xLocState: {blobXLocationState}, Distance: {blobDistanceState}, Combined State: {state}")
            self.rvr.led_control.set_all_leds_rgb(red=255, green=165, blue=0)
        return state

    def get_beacon(self, videoGetter):
        """
        Gets a video frame, searches for blobs in the frame using the detector.
        """
        # Gets frame from videoGetter obj
        frame = videoGetter.frame
        avgXY, avgSize = self.beacon_detector.blob_detector(frame)

        # If no blob, return nothing
        if np.isnan(avgXY[0]) and np.isnan(avgSize):
            return None,None

        # return data about the blob
        return avgXY, avgSize

    def step(self, action, videoGetter):
        # Send action command to robot and get next state.
        driveParams = self.actions[action]
        self.colorFlag = asyncio.run(driver(*driveParams))
        print(f"Color Flag: {self.colorFlag}")
        print(f"Driving: {driveParams[-1]}")
        print(f"Reward State: {self.reward_state}")
        new_state = self.get_state(videoGetter)
        
        # Resets the colorFlag after setting the state

        reward = 0
        completeStatus = False
        # Debugging: print(f"Before if/else in Step(): New State: {new_state}, Reward: {reward}, Complete Status {completeStatus}")
        if (new_state == self.reward_state):
            reward = 1
            completeStatus = True
            # reached goal. done, leds set to green
            self.colorFlag = True
            self.rvr.led_control.set_all_leds_rgb(red=0, green=255, blue=0)
        elif (new_state == self.failState):
            reward = 0
            completeStatus = True
            # failed. done, leds set to red
            self.rvr.led_control.set_all_leds_rgb(red=255, green=0, blue=0)
        else:
            reward = 0
            # Not done, still searching, leds set to orange
            self.rvr.led_control.set_all_leds_rgb(red=255, green=165, blue=0)
        # Debugging: print(f"End of Step(): New State: {new_state}, Reward: {reward}, Complete Status {completeStatus}")
        return new_state, reward, completeStatus


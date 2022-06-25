from colorsys import rgb_to_hls
import os, sys, asyncio, numpy as np, cv2 as cv, random
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../sphero-sdk-raspberrypi-python/')))
from sphero_sdk import SpheroRvrObserver, Colors, RvrStreamingServices
from videoGet import VideoGet

from drive import driver, color_detected_handler
from detector_class import Detector, pink_ball, blue_bucket

class ServoingEnvironment:

    def __init__(self) -> None:
        
        # Creates RVR object
        self.rvr = SpheroRvrObserver()
        
        # Enables the rovers bottom facing color sensor
        self.rvr.enable_color_detection(is_enabled=True)
        
        # Creates the handler for the RVR's color detection
        self.rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.color_detection,
            handler=color_detected_handler
        )

        # Creates flag for when RVR detects a specific color with it's sensor
        self.colorFlag = False

        # Lists out actions that the rover can take.
        # NOTE: Legend: [rvrObject, L-trk drive mode, R-trk drive mode,.. 
        # ..drive time (seconds), speed, command name, led color code]
        # Drive Modes: 0 - Stop, 1 - Forward, 2 - Reverse
        self.actions = []
        self.actions.append([self.rvr, 1, 2, .15, 180, "Hard Right"])
        self.actions.append([self.rvr, 1, 2, .1, 180, "Right"])
        self.actions.append([self.rvr, 1, 2, .05, 180, "Soft Right"])
        #self.actions.append([self.rvr, 1, 1, .25, 180, "Forward"])
        #self.actions.append([self.rvr, 2, 2, .25, 180, "Reverse"])
        self.actions.append([self.rvr, 2, 1, .05, 180, "Soft Left"])
        self.actions.append([self.rvr, 2, 1, .1, 180, "Left"])
        self.actions.append([self.rvr, 2, 1, .15, 180, "Hard Left"])

        # Gets number of possible actions
        self.numActions = len(self.actions)
        self.image_divisions = 7
        
        # Image width of video frame, can be found using OpenCV
        self.image_width = 640

        # Used for returning the blob width state, which estimates how close a blob is
        self.blobSizeStateDict = {20:0, 40:1, 60:2, 80:3, 90:4, 999999:5}

        # Calculates total possible states, additional state is for no blob detected
        self.numStates = (self.image_divisions * len(self.blobSizeStateDict)) + 1

        # signifies blob is not in the visual field
        self.failState = self.numStates - 1

        # Gets middle slice of image divisions for reward state
        self.reward_state = (((self.image_divisions)//2) * len(self.blobSizeStateDict)) + max(self.blobSizeStateDict.values())
        self.current_state = self.failState

        # Creates a detector for blob detection of target object
        self.target_detector = Detector(pink_ball)

        # Creates a detector for blob detection of beacon object
        self.beacon_detector = Detector(blue_bucket)

    def randomRestart(self, rvr):
        # Generates a random-ish drive command for restarting the test
        restartLeftTrack = random.randint(1,2)
        if restartLeftTrack == 1:
            restartRightTrack = 2
        else:
            restartRightTrack = 1
        restartDrive = [rvr, restartLeftTrack, restartRightTrack, 
        random.uniform(.1,.5), 180, "Restart", [255,0,255]]

        return restartDrive, restartLeftTrack, restartRightTrack

    def reset_with_beacon(self, videoGetter):
        self.locate_furthest_beacon(videoGetter)
        
        # visually servo to beacon
        xy, size = self.get_beacon(videoGetter)
        print(f'Ready to servo to {size}')
        input('pause')
        while size == None:
            print('acquiring beacon for servoing')
            # it is losing the beacon
            xy, size = self.get_beacon(videoGetter)

        while size < 75:
            input(f'in while with size {size}')
            input('pause')
            # determine % from center of frame (signed)
            # -1 is at left edge and +1 at right, 0 in middle
            error_from_center = xy[0] / (self.image_width/2) - 1

            err = abs(error_from_center)
            time = -.09*pow(err,3) + .19*pow(err,2) + .09*err + .07
            print(f'time {time}')
            if err <= .01:    # go straight
                asyncio.run(driver(*[self.rvr, 1, 1, time+.05, 180, "beacon"]))
            elif error_from_center < 0: # need to turn left
                asyncio.run(driver(*[self.rvr, 2, 1, time, 180, "beacon"]))
            else: # need to turn right
                asyncio.run(driver(*[self.rvr, 1, 2, time, 180, "beacon"]))
            
            xy, size = self.get_beacon(videoGetter)
            while size == None:
                xy, size = self.get_beacon(videoGetter)
                print('acquiring beacon')
            print(f'after servo to {xy,size}')
            input('pause')

        # randomly move about to start
        self.reset(videoGetter)

    def locate_furthest_beacon(self,videoGetter):
        '''
        locate beacon farthest from current position. 
        rotate in a circle until both beacons are located.
        determine which is further, 
        rotating back if necessary to make sure the beacon is in frame
        '''

        restartDriveCommand, restartLeftTrack, restartRightTrack = self.randomRestart(self.rvr)
        
        # rotate until a beacon is found
        print('@@@ Rotating until first beacon is found')
        input('pause')
        size_first = None
        while size_first == None:
            xy, size_first = self.get_beacon(videoGetter)
            if size_first != None:
                break
            asyncio.run(driver(*[self.rvr, restartLeftTrack, restartRightTrack, .2, 180, "Scanning"]))

        # keep rotating until the beacon is not found
        print(f'111 Rotating until first beacon is lost. size {size_first}')
        input('pause')
        size = size_first
        while size != None:
            xy, size = self.get_beacon(videoGetter)
            if size == None:
                break
            asyncio.run(driver(*[self.rvr, restartLeftTrack, restartRightTrack, .2, 180, "Scanning"]))

        # keep rotating until the 2nd beacon is found
        print('222 Rotating until 2nd beacon is found')
        input('pause')
        size_second = None
        while size_second == None:
            xy, size_second = self.get_beacon(videoGetter)
            if size_second != None:
                break
            asyncio.run(driver(*[self.rvr, restartLeftTrack, restartRightTrack, .2, 180, "Scanning"]))

        # compare sizes and rotate to the smallest one
        if size_first < size_second:
            print('Going back to first beacon')
            input('pause')
            # reverse rotate back to first beacon
            # lets turn a bit to make sure not seeing second beacon first
            # exchanging the directions of RightTrack and LeftTrack
            asyncio.run(driver(*[self.rvr, restartRightTrack, restartLeftTrack, .2, 180, "Scanning"]))
            asyncio.run(driver(*[self.rvr, restartRightTrack, restartLeftTrack, .2, 180, "Scanning"]))

            # rotate until back to first beacon
            size = None
            while size == None:
                xy, size = self.get_beacon(videoGetter)
                if size != None:
                    break
            asyncio.run(driver(*[self.rvr, restartRightTrack, restartLeftTrack, .2, 180, "Scanning"]))

    
    def reset(self, videoGetter):
        """This has the RVR place itself into a somewhat-random starting position by rotating"""
        print("RVR being reset")
        state = self.failState
        restartDriveCommand, restartLeftTrack, restartRightTrack = self.randomRestart(self.rvr)
        asyncio.run(driver(*restartDriveCommand))
        
        # The RVR will rotate until it sees a blob. (Search mode)
        while state == self.failState:
            state = self.get_state(videoGetter)
            if state != self.failState:
                break
            asyncio.run(driver(*[self.rvr, restartLeftTrack, restartRightTrack, .2, 180, "Scanning"]))
        return state
    
    def get_state(self, videoGetter):
        """Gets a video frame, searches for blobs in the frame using the detector,
        """
        # Gets frame from videoGetter obj
        frame = videoGetter.frame
        avgXY, avgSize = self.target_detector.blob_detector(frame)
        # testing if beacon detector was working
        #avgXY, avgSize = self.beacon_detector.blob_detector(frame)
        
        # Draws circle on blob location, if available
        if not np.isnan(avgXY[0]):
            frame = cv.circle(frame, (int(avgXY[0]), int(avgXY[1])), int(avgSize//2), (0,255,0), 3)
        
        # Displays the frame in a new window
        cv.imshow('Frame', frame)

        # Returns state dependent on if there is a blob on frame or not, and
        # whether or not the RVR drove over a color that is not allowed
        if self.colorFlag or (np.isnan(avgXY[0]) and np.isnan(avgSize)):
            state = self.failState   
        else:
            # Gets the state of the x-coordinate of the blob
            blobXLocationState = int(avgXY[0]*self.image_divisions/self.image_width)
            # Gets the percentage size (blobRatio) of the blob, compared to the frame size (width)
            blobRatio = int((avgSize/self.image_width)*100)
            # Checks dictionary to assign blobSizeState to a number based on the blobRatio
            for size in self.blobSizeStateDict:
                if blobRatio < size:
                    blobSizeState = self.blobSizeStateDict[size]
                    break
            # Combines the blob x-coordinate state and blob size into a single state
            state = (blobXLocationState * len(self.blobSizeStateDict)) + blobSizeState
            self.rvr.led_control.set_all_leds_rgb(red=255, green=165, blue=0)
        return state

    def get_beacon(self, videoGetter):
        """Gets a video frame, searches for blobs in the frame using the detector,
        """
        # Gets frame from videoGetter obj
        frame = videoGetter.frame
        avgXY, avgSize = self.beacon_detector.blob_detector(frame)

        cv.imshow('Frame',frame)

        # If no blob, return nothing
        if np.isnan(avgXY[0]) and np.isnan(avgSize):
            return None,None
        
        # Draws circle on blob location and shows in window
        frame = cv.circle(frame, (int(avgXY[0]), int(avgXY[1])), int(avgSize//2), (0,255,0), 3)
        cv.imshow('Frame', frame)

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
        self.colorFlag = False

        reward = 0
        completeStatus = False

        if (new_state == self.reward_state):
            reward = 1
            completeStatus = True
            # reached goal. done, leds set to green
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
        return new_state, reward, completeStatus


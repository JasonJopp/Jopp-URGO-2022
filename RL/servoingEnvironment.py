from colorsys import rgb_to_hls
import os, sys, asyncio, numpy as np, cv2 as cv, random
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),
    '../../sphero-sdk-raspberrypi-python/')))
from sphero_sdk import SpheroRvrObserver, Colors, RvrStreamingServices
from videoGet import VideoGet

from drive import driver, color_detected_handler
from detector_class import Detector, pink_box, blue_bucket

class ServoingEnvironment:

    def __init__(self) -> None:
        
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
            [self.rvr, 1, 1, .4, 160, "Fast Forward"],
            [self.rvr, 1, 1, .2, 160, "Forward"],
            [self.rvr, 1, 1, .1, 160, "Slow Forward"],
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

        # This is used when assigning the blob width state. Keys are used to
        # measure blob diameter compared to frame (%), values are the states
        #self.blobSizeState = {10:0, 20:1, 90:2, 999999:3} # pink_ball
        self.blobSizeStateList = [[20,0], [40,1], [95,2], [110,3], [999999999,4]] # green_box/pink_box
        
        # Sets number of possible states, added state is for no blob detected
        self.numStates = (self.image_divisions * len(self.blobSizeStateList)) + 1

        # Signifies that blob left RVR's view, or if RVR moved out of bounds
        self.failState = self.numStates - 1

        # Gets middle slice of image divisions for reward state
        self.reward_state = (((self.image_divisions)//2) * \
        len(self.blobSizeStateList)) + self.blobSizeStateList[-2][1]
        
        self.current_state = self.failState

        # Creates a detector for blob detection of target object
        self.target_detector = Detector(pink_box)

        # Creates a detector for blob detection of beacon object
        self.beacon_detector = Detector(blue_bucket)

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
            print(f'in while with size {size}')
            # determine % from center of frame (signed)
            # -1 is at left edge and +1 at right, 0 in middle
            error_from_center = xy[0] / (self.image_width/2) - 1
            err = abs(error_from_center)
            time = .13*pow(err,3) + -.3*pow(err,2) + .33*err + -.01
            print(f'time {time}')
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
            print(f'after servo to {xy,size}')

    def locate_furthest_beacon(self,videoGetter):
        '''
        locate beacon farthest from current position. 
        rotate in a circle until both beacons are located.
        determine which is further, 
        rotating back if necessary to make sure the beacon is in frame
        '''

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
            print(f"Average Size: {avgSize}")
            print(f"Blob Ratio: {blobRatio}")

            # Checks blobSizeStateList to assign blobSizeState to a number based on the blobRatio
            for blobSizeBin in self.blobSizeStateList:
                if blobSizeBin[0] > blobRatio:
                    self.blobSizeState = blobSizeBin[1]
                    break
            # Combines the blob x-coordinate state and blob size into a single state
            state = (blobXLocationState * len(self.blobSizeStateList)) + self.blobSizeState
            print(f"xLocState: {blobXLocationState}, Blob Size State: {self.blobSizeState}, Combined State: {state}")
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
        print(f"Before if/else in Step(): New State: {new_state}, Reward: {reward}, Complete Status {completeStatus}")
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
        print(f"End of Step(): New State: {new_state}, Reward: {reward}, Complete Status {completeStatus}")
        return new_state, reward, completeStatus


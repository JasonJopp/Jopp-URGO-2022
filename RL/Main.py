import cv2 as cv
from VideoGet import VideoGet

def threadVideoGet(source = 0):
    """
    Dedicated thread for grabbing video frames with VideoGet object.
    Main thread shows video frames.
    """

    video_getter = VideoGet(source).start()
    
    while True:
        frame = video_getter.frame

def main():
    threadVideoGet()

if __name__ == "__main__":
    main()
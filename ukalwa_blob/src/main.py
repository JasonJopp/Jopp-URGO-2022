from threading import Thread
from queue import Queue
from capture_video import create_video_feed
from BGR_color_filter import create_BGR_filter
import capture_video


if __name__ == "__main__":

	# Creates threads list, adds each thread to the list
	threads = []
	threads.append(Thread(target=create_video_feed))
	threads.append(Thread(target=create_BGR_filter))
	
	# Starts threads
	for thread in threads:
		thread.start()

	# Joins threads
	for thread in threads:
		thread.join()

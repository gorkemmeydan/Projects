"""
Captures video from the Rasberry pi camera and writes it to the SD card
placed on it. 

Görkem Meydan 2018
"""


# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import os

pixel_size = (640,480)
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = pixel_size
camera.framerate = 24
rawCapture = PiRGBArray(camera, size=pixel_size)

date_and_time = time.strftime("%c").replace(":", ".")
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(os.path.join('/home/pi/Desktop/IKA_video',"ika_yol_{}.avi".format(date_and_time)),fourcc, 20.0, (640,480))
# allow the camera to warmup
time.sleep(0.1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
 
	# show the frame
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF
	out.write(image)
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
cv2.destroyAllWindows()
out.release()
os.system("sudo cp -R /home/pi/Desktop/IKA_video /boot")
import picamera
from picamera.array import PiRGBArray
import numpy as np
import cv2
import time 
import serial
import RPi.GPIO as GPIO

#communication with arduino

ser = serial.Serial(
	port = '/dev/ttyACM0',
	baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
       	timeout=1
)

State = 0
pre_State = 0

# Parameter definition
NONE       = 0
ON_AND_OFF = 1 # up
BIGGER     = 2 # left
SMALLER    = 3 # right
ROTATION   = 4 # down
AUTO       = 5

LED1 = 13
LED2 = 26

#setup camera & GPIO
camera = picamera.PiCamera()
rawCapture = PiRGBArray(camera)
time.sleep(0.1)

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED1, GPIO.OUT)
GPIO.setup(LED2, GPIO.OUT)

GPIO.output(LED1, 0)
GPIO.output(LED2, 0)

camera.start_preview()

pre_x = -1
pre_y = -1
pre_area = -1
pre_hsv = 0

def Write_to_Arduino():
    global State
    if State == ON_AND_OFF:

	ser.write('1')
    elif State == AUTO:

	ser.write('5')
    elif State == BIGGER:

	ser.write('3')
    elif State == SMALLER:

	ser.write('4')
    elif State == ROTATION:

	ser.write('2')
    else:
	return -1
    time.sleep(1)
    return -1

while True:
    try:
	#capture image from camera
	camera.capture(rawCapture, format='bgr', use_video_port=True)
	image = rawCapture.array

	#convert to hsv and blur it
	aft_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	skinImg = cv2.inRange(aft_hsv, (0, 75, 50), (40, 140, 255))
	thre = skinImg
	thre = cv2.erode(thre, None, iterations=2)
	(cnts, _) = cv2.findContours(skinImg, cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)

	height, width = skinImg.shape[:2]
	black = np.zeros((height,width,3), np.uint8)


	# loop over the contours	
	count = 0
	if State == AUTO:
	    pre_State = AUTO
	else:
	    pre_State = NONE	

	for c in cnts:
		# if the contour is too small, ignore it
		if cv2.arcLength(c, True) < 2000:
		    continue
		count += 1
		
		#draw all contour
		cv2.drawContours(black, [c], 0, (255, 255, 255), -1)

		#find rotate rectangles with minimum area
		rect = cv2.minAreaRect(c)
		box = cv2.cv.BoxPoints(rect)
		box = np.int0(box)
		position = (box[0] + box[1] + box[2] + box[3]) / 4

		if pre_x != -1:
		    #def 
		    diff_x = pre_x - position[0]
		    diff_y = pre_y - position[1]
		    if abs(diff_x) >= 60 or abs(diff_y) >= 50: 
			if abs(diff_x) > 1.2*abs(diff_y):#right or left
		    	    if diff_x > 0:
				if State != AUTO:
			    	    State = BIGGER
			    	time.sleep(0.1)

		    	    elif diff_x < 0:
				if State != AUTO:
			    	    State = SMALLER
			    	time.sleep(0.1)

		    	elif diff_y > 0:
			    if State != AUTO:
			        State = ROTATION
			    time.sleep(0.5)

		    	elif diff_y < 0:
			    State = ON_AND_OFF
			    time.sleep(0.5)

		    elif cv2.contourArea(c) > 1.25* pre_area:

			if State != AUTO:
			    State = AUTO
			else:
			    State = NONE
			time.sleep(0.5)

		    elif cv2.contourArea(c) < 0.8* pre_area:

			if State != AUTO:
			    State = AUTO
			else : 
			    State = NONE
			time.sleep(0.5)

		    else:
			State = NONE
		cv2.drawContours(image, [box],0,(0, 255, 0),3)
		pre_x = position[0]
		pre_y = position[1]
		pre_area = cv2.contourArea(c)

	if count != 1:

	    pre_x = -1
	    pre_y = -1
	    pre_area = -1
	    State = pre_State
	    if count == 0:
		GPIO.output(LED1, 0)
		GPIO.output(LED2, 0)
	    else:
		GPIO.output(LED1, 1)
		GPIO.output(LED2, 1)

        if State == AUTO:
            GPIO.output(LED1, 0)
            GPIO.output(LED2, 1)
            Write_to_Arduino()

            time.sleep(1)
	elif count == 1:
	    GPIO.output(LED1, 1)
	    GPIO.output(LED2, 0)
	    Write_to_Arduino()

	    time.sleep(1)
	pre_hsv = aft_hsv

	cv2.waitKey(5)
	rawCapture.seek(0)
	rawCapture.truncate()

#    except IOError:
#	continue

    except KeyboardInterrupt:
	camera.stop_preview()
	camera.close()
	cv2.destroyAllWindows()
	GPIO.output(LED1, 0)
	GPIO.output(LED2, 0)
	break


import cv2
from PIL import Image
import numpy as np
import RPi.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BOARD)



def pwm_control():
    GPIO.setup(11, GPIO.OUT)
    p = GPIO.PWM(11, 50)
    p.start(0)

    p.ChangeDutyCycle(3)
    sleep(1)
    p.ChangeDutyCycle(12)
    sleep(1)

    p.stop()
    GPIO.cleanup()


red = [0, 0, 255]
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()

    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lowerLimit =  ([0, 160,160])
    upperLimit =  ([10,255,255])
    lowerLimit = np.array(lowerLimit,dtype=np.uint8)
    upperLimit = np.array(upperLimit,dtype=np.uint8)
    mask0=cv2.inRange(hsvImage,lowerLimit,upperLimit)



    lowerLimit =  ([175,160,160])
    upperLimit =  ([180,255,255])
    lowerLimit = np.array(lowerLimit,dtype=np.uint8)
    upperLimit = np.array(upperLimit,dtype=np.uint8)
    mask1 = cv2.inRange(hsvImage,lowerLimit,upperLimit)

    mask=mask0+ mask1

    mask_ = Image.fromarray(mask)

    bbox = mask_.getbbox()

    if bbox is not None:
        x1, y1, x2, y2 = bbox

        frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
        pwm_control(11,50,3,12,1)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
          break

cap.release()

cv2.destroyAllWindows()
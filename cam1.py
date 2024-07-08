import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from time import sleep

GPIO.setmode(GPIO.BOARD)

def pwm_control(pin, frequency, duty_cycle_start, duty_cycle_end, delay):
    GPIO.setup(pin, GPIO.OUT)
    p = GPIO.PWM(pin, frequency)
    p.start(0)

    p.ChangeDutyCycle(duty_cycle_start)
    sleep(delay)
    p.ChangeDutyCycle(duty_cycle_end)
    sleep(delay)

    p.stop()
    GPIO.cleanup()

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

try:
    while True:
        frame = picam2.capture_array()

        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lowerLimit = np.array([0, 120, 70], dtype=np.uint8)
        upperLimit = np.array([10, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y, w, h) = cv2.boundingRect(largest_contour)
            frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)

            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(frame, (cX, cY), 5, (0, 255, 0), -1)

            pwm_control(11, 50, 3, 12, 1)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    picam2.stop()
    cv2.destroyAllWindows()

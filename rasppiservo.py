import cv2
import numpy as np
from PIL import Image
import RPi.GPIO as GPIO  
from time import sleep   
GPIO.setmode(GPIO.BOARD) 

def pwm_control(pin_number,frequency,duty_cycle_start,duty_cycle_end,duration):
    GPIO.setmode(GPIO.BOARD)  
    GPIO.setup(pin_number, GPIO.OUT)  
    p = GPIO.PWM(pin_number, frequency)  
    p.start(duty_cycle_start)  

    try:
        p.ChangeDutyCycle(duty_cycle_start)
        sleep(duration)
        p.ChangeDutyCycle(duty_cycle_end)
        sleep(duration)

    except KeyboardInterrupt:
        p.stop()
        GPIO.cleanup()




yellow= [0,255,255]
cap=cv2.VideoCapture(0)
while True:
    ret,frame=cap.read()

    hsvImage=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    lowerLimit=([10,100,100])
    upperLimit=([20,255,255])

    lowerLimit=np.array(lowerLimit, dtype=np.uint8)
    upperLimit=np.array(upperLimit, dtype=np.uint8)

    mask=cv2.inRange(hsvImage,lowerLimit,upperLimit)
    mask_=Image.fromarray(mask)

    bbox=mask_.getbbox()

    if bbox is not None:
        x1,y1,x2,y2 =bbox

        frame=cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,255),5)
        pwm_control(11,50,3,12,1)

    cv2.imshow('frame',frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
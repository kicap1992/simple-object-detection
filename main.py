import cv2
# import RPi.GPIO as GPIO
import time
import numpy as np
import serial


# GPIO.setmode(GPIO.BOARD)
# # set the servo on 11
# GPIO.setup(11, GPIO.OUT)

# # set the servo on 11 to 50Hz
# p = GPIO.PWM(11, 50)
# p.start(0)

# # set the servo to 0 degrees

# position = 90

# # create a function to set the angle of the servo
# def SetAngle(angle):
#     duty = angle / 18 + 2
#     GPIO.output(11, True)
#     p.ChangeDutyCycle(duty)
#     time.sleep(1)
#     GPIO.output(11, False)
#     p.ChangeDutyCycle(0)

# # set the servo to 0 degrees
# SetAngle(0)

# # set the servo to 90 degrees
# SetAngle(90)

cap = cv2.VideoCapture(0)
# set the width and height to 320x240
cap.set(3, 320)
cap.set(4, 240)
_, frame = cap.read()
rows, cols, _ = frame.shape
x_medium = int(cols / 2)
center = int(cols / 2)

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.flush()

while True:
    ret, frame = cap.read()
    low_blue = np.array([100, 150, 0])
    high_blue = np.array([140, 255, 255])

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low_blue, high_blue)

    

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)

    for cnt in contours:
        (x, y, w, h) = cv2.boundingRect(cnt)
        x_medium = int((x + x + w) / 2)
        break

    cv2.line(frame, (x_medium, 0), (x_medium, 480), (0, 255, 0), 2)
    # cv2.imshow("Mask", mask)
    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    if x_medium < center  - 25:
        # position += 1
        # if position > 180:
        #     position = 180
        # SetAngle(position)
        ser.write(b'right\n')
        # time.sleep(0.5)
        print("Servo is moving to the right")
    elif x_medium > center + 25:
        # position -= 1
        # if position < 0:
        #     position = 0
        # SetAngle(position)
        ser.write(b'left\n')
        # time.sleep(0.5)
        print("Servo is moving to the left")
    else:
        ser.write(b'stop\n')
        # time.sleep(0.5)
        print("Servo is not moving")

            

cap.release()
cv2.destroyAllWindows()
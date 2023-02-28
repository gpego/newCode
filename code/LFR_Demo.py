import cv2
import numpy as np
from Squares_Detection import Detect_Squares
import time
#import RPi.GPIO as GPIO

#cap = cv2.VideoCapture(1)
#cap = cv2.VideoCapture(r"C:/Users/giovi/Desktop/RoboCup/Sync2/newCode/vids/VideoCapture01.mp4")
cap = cv2.VideoCapture(r"C:\Users\giovi\Desktop\RoboCup\Sync2\newCode\vids\2023-02-20 ore 18.57.44_mod.mp4")

maxSpeed = 40
# Greater curveSens --> faster turn
curveSens = 4   # Min. value for curveSens = 2

"""
EnaA, In1A, In2A, EnaB, In1B, In2B
frontMotors = Motor(25, 24, 23, 27, 9, 22)
backMotors = Motor(19, 26, 13, 12, 6, 5)

enA = 25

in1 = 4
in2 = 17
in3 = 27
in4 = 22
en1 = 23
en2 = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(en1, GPIO.OUT)
GPIO.setup(en2, GPIO.OUT)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
p1 = GPIO.PWM(en1, 100)
p2 = GPIO.PWM(en2, 100)
p1.start(50)
p2.start(50)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)
"""

prev_frame_time = 0
new_frame_time = 0

while True:
    ret, img = cap.read()

    frame = cv2.resize(img,(64, 64))

    # IMAGE PRE-PROCESSING

    # Convert to gray scale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Blur the image
    blur = cv2.blur(gray, (5,5))

    # Searching for green sqares
    sq_mask = Detect_Squares(frame)

    # Line detection
    th3 = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 51, 8)

    # Removing green sqaures area from the track mask
    new_mask = cv2.subtract(th3, sq_mask)

    # Detecting track contours
    contours, hierarchy = cv2.findContours(new_mask, 1, cv2.CHAIN_APPROX_NONE)

    # Checking for contours' availability
    if len(contours) > 0 :
        c = max(contours, key=cv2.contourArea)  # Keeping the biggest contour
        M = cv2.moments(c)

        cv2.drawContours(frame, c, -1, (0,255,0), 1)    # Drawing contours on the frame

        if M["m00"] !=0 :
            # Calculating countours' center point
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            print("CX : "+str(cx)+"  CY : "+str(cy))

            # MOTORS CONTROL

            # cx > (32 + 8 / 2)
            if cx > 36:
                # Far-right
                if cx >= 48:
                    speed = cx - 48 # Range 0 - 16

                    leftSpeed = 16 - int(speed / curveSens)
                    rightSpeed = leftSpeed

                    leftSpeed = leftSpeed / 16 * maxSpeed
                    rightSpeed = rightSpeed / 16 * maxSpeed

                    # INVERTIRE SENSO DI ROTAZIONE MOTORI!!!!!!!

                # Right
                else:
                    speed = cx - 32 # Range 0 - 16

                    leftSpeed = 16 - int(speed / curveSens)
                    rightSpeed = 16 - speed

                    leftSpeed = leftSpeed / 16 * maxSpeed
                    rightSpeed = rightSpeed / 16 * maxSpeed

                    #STESSO SENSO DI ROTAZIONE!!!!!!!!
            
            # cx < (32 - 8 / 4)
            if cx < 28:
                # Far-left
                if cx <= 16:
                    speed = cx # Range 0 - 16

                    rightSpeed = 16 - int(speed / curveSens)
                    leftSpeed = rightSpeed

                    rightSpeed = rightSpeed / 16 * maxSpeed
                    leftSpeed = leftSpeed / 16 * maxSpeed

                    # INVERTIRE SENSO DI ROTAZIONE MOTORI!!!!!!!

                # Left
                else:
                    speed = cx - 16 # Range 0 - 16

                    rightSpeed = int(speed - curveSens)
                    leftSpeed = 16 - speed

                    rightSpeed = rightSpeed / 16 * maxSpeed
                    leftSpeed = leftSpeed / 16 * maxSpeed

                    #STESSO SENSO DI ROTAZIONE!!!!!!!!

                    
        cv2.circle(frame, (cx,cy), 5, (255,255,255), -1)    # Drawing centerpoint on the frame
    else :
        print("I don't see the line")
        #motori avanti maxSpeed, stesso senso di rotazione

    # Video output
    cv2.imshow("Mask", new_mask)
    cv2.imshow("Frame", cv2.resize(frame, (240,240)))

    #FPS count
    new_frame_time = time.time()

    fps = 1/(new_frame_time-prev_frame_time)
    prev_frame_time = new_frame_time

    print(f"FPS = {fps}")

    # Exiting the loop
    if cv2.waitKey(1) & 0xff == ord('q'):   # 1 is the time in ms
        """
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.LOW)
        """
        break
cap.release()
cv2.destroyAllWindows()
import cv2
import numpy as np
#import RPi.GPIO as GPIO
#cap = cv2.VideoCapture(1)
#cap = cv2.VideoCapture(r"C:/Users/giovi/Desktop/RoboCup/Sync2/newCode/vids/VideoCapture01.mp4")
cap = cv2.VideoCapture(r"C:/Users/giovi/Desktop/RoboCup/Sync2/newCode/vids/VideoCapture01-flipped.mp4")

font = cv2.FONT_HERSHEY_COMPLEX

"""
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
while True:
    ret, img = cap.read()

    frame = cv2.resize(img,(160, 120))

    low_b = np.uint8([179, 70, 255])
    high_b = np.uint8([0, 0, 0])
    mask = cv2.inRange(frame, high_b, low_b)

    contours, hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)


    # Going through every contours found in the image.
    for cnt in contours :

        approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)

        # draws boundary of contours.
        cv2.drawContours(frame, [approx], 0, (0, 0, 255), 5)

        # Used to flatted the array containing
        # the co-ordinates of the vertices.
        n = approx.ravel()
        i = 0

        for j in n :
            if(i % 2 == 0):
                x = n[i]
                y = n[i + 1]

                # String containing the co-ordinates.
                string = str(x) + " " + str(y)

                if(i == 0):
                    # text on topmost co-ordinate.
                    cv2.putText(frame, "Arrow tip", (x, y),
                                    font, 0.5, (255, 0, 0))
                else:
                    # text on remaining co-ordinates.
                    cv2.putText(frame, string, (x, y),
                            font, 0.5, (0, 255, 0))
            i = i + 1


    if len(contours) > 0 :
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)

        if M["m00"] !=0 :
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print("CX : "+str(cx)+"  CY : "+str(cy))

            if cx >= 120 :
                print("Turn Left")
                
            if cx < 120 and cx > 40 :
                print("On Track!")
                
            if cx <=40 :
                print("Turn Right")
                
            cv2.circle(frame, (cx,cy), 5, (255,255,255), -1)
    else :
        print("I don't see the line")
        

    #cv2.drawContours(frame, c, -1, (0,255,0), 1)
    #cv2.imshow("Mask",mask)
    cv2.imshow("Frame",frame)

    """
    ###################################################
    # Going through every contours found in the image.
    for cnt in contours :

        approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)

        # draws boundary of contours.
        cv2.drawContours(img, [approx], 0, (0, 0, 255), 5)

        # Used to flatted the array containing
        # the co-ordinates of the vertices.
        n = approx.ravel()
        i = 0

        for j in n :
            if(i % 2 == 0):
                x = n[i]
                y = n[i + 1]

                # String containing the co-ordinates.
                string = str(x) + " " + str(y)

                if(i == 0):
                    # text on topmost co-ordinate.
                    cv2.putText(img, "Arrow tip", (x, y),
                                    font, 0.5, (255, 0, 0))
                else:
                    # text on remaining co-ordinates.
                    cv2.putText(img, string, (x, y),
                            font, 0.5, (0, 255, 0))
            i = i + 1

    # Showing the final image.
    cv2.imshow('image2', img)
    ###################################################
    """

    if cv2.waitKey(0) & 0xff == ord('q'):   # 1 is the time in ms
        """
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.LOW)
        """
        break
cap.release()
cv2.destroyAllWindows()
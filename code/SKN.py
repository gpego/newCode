import cv2
import numpy as np

#Load the image in grayscale
cap = cv2.VideoCapture(r"C:/Users/giovi/Desktop/RoboCup/Sync2/newCode/vids/VideoCapture01-flipped.mp4")

while True:
    ret, frame = cap.read()
    
    #Threshold the image with threshold value 127
    low_b = np.uint8([179, 70, 255])
    high_b = np.uint8([0, 0, 0])
    img = cv2.inRange(frame, high_b, low_b)

    # Step 1: Create an empty skeleton
    size = np.size(img)
    skel = np.zeros(img.shape, np.uint8)

    # Get a Cross Shaped Kernel
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))

    # Repeat steps 2-4
    while True:
        #Step 2: Open the image
        open = cv2.morphologyEx(img, cv2.MORPH_OPEN, element)
        #Step 3: Substract open from the original image
        temp = cv2.subtract(img, open)
        #Step 4: Erode the original image and refine the skeleton
        eroded = cv2.erode(img, element)
        skel = cv2.bitwise_or(skel,temp)
        img = eroded.copy()
        # Step 5: If there are no white pixels left ie.. the image has been completely eroded, quit the loop
        if cv2.countNonZero(img)==0:
            break

    # Displaying the final skeleton
    cv2.imshow("Skeleton",skel)
    cv2.imshow("Vid", frame)
    cv2.waitKey(1)
    if cv2.waitKey(1) & 0xff == ord('q'):   # 1 is the time in ms
            break
cap.release()
cv2.destroyAllWindows()
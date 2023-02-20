import cv2
import numpy as np

#cap = cv2.VideoCapture(r"C:/Users/giovi/Desktop/RoboCup/Sync2/newCode/vids/VideoCapture01.mp4")
cap = cv2.VideoCapture(r"C:/Users/giovi/Desktop/RoboCup/Sync2/newCode/vids/VideoCapture01-flipped.mp4")

while True:
    ret, img = cap.read()
    #gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    """
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size),0)
    """
    low_b = np.uint8([179, 70, 255])
    high_b = np.uint8([0, 0, 0])
    mask = cv2.inRange(img, high_b, low_b)

    low_threshold = 10
    high_threshold = 240
    edges = cv2.Canny(mask, low_threshold, high_threshold)

    rho = 1  # distance resolution in pixels of the Hough grid
    theta = np.pi / 180  # angular resolution in radians of the Hough grid
    threshold = 15  # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 80  # minimum number of pixels making up a line
    max_line_gap = 100  # maximum gap in pixels between connectable line segments
    line_image = np.copy(img) * 0  # creating a blank to draw lines on

    # Run Hough on edge detected image
    # Output "lines" is an array containing endpoints of detected line segments
    lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                        min_line_length, max_line_gap)

    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),5)

    # Draw the lines on the  image
    lines_edges = cv2.addWeighted(img, 0.8, line_image, 1, 0)

    cv2.imshow("Img", lines_edges)

    if cv2.waitKey(1) & 0xff == ord('q'):   # 1 is the time in ms
        break
cap.release()
cv2.destroyAllWindows()
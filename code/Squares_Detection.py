import cv2
import numpy as np

def Detect_Squares(frame):
    # Convert to HSV colorspace
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # lower bound and upper bound for Green color
    lower_bound = np.array([50, 20, 20])	 
    upper_bound = np.array([100, 255, 255])

    # find the colors within the boundaries
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    #define kernel size  
    kernel = np.ones((7,7),np.uint8)

    # Remove unnecessary noise from mask

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    return mask





if __name__ == "__main__":
    cap = cv2.VideoCapture(r"C:\Users\giovi\Desktop\RoboCup\Sync2\newCode\vids\2023-02-20 ore 18.57.44_mod.mp4")

    while True:
        ret, frame = cap.read()

        img = cv2.resize(frame,(160, 120))

        mask = Detect_Squares(img)

        # Find contours from the mask

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        output = cv2.drawContours(img, contours, -1, (0, 0, 255), 3)

        # Showing the output

        cv2.imshow("Output", output)

        cv2.imshow("Squares", mask)
        
        if cv2.waitKey(1) & 0xff == ord('q'):   # 1 is the time in ms
            break
    cap.release()
    cv2.destroyAllWindows()
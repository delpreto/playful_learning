import numpy as np
import cv2
import sys

videoCapture = cv2.VideoCapture(sys.argv[1])

count = 0
while(True):
    # Capture frame-by-frame
    ret, frame = videoCapture.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if count == 0:
	    cv2.imwrite('/home/drl/ros_ws/src/baxter_screen/test.jpg', frame)
	    count = count+1

# When everything done, release the capture
videoCapture.release()
cv2.destroyAllWindows()

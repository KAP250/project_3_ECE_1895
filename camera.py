import cv2
import time
import imutils
import _thread
import threading
import atexit
import sys
import termios
import contextlib

video_capture = cv2.VideoCapture(0)
while True:
    # Capture frame-by-frame
    ret, frame = video_capture.read()

    # Display the resulting frame
    cv2.imshow('Video', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
video_capture.release()
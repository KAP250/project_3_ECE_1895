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



#Code used myself to test motion detection
# camera = cv2.VideoCapture(camera_port)
# camera.set(3, 320)
# camera.set(4, 240)
# camera.set(cv2.CAP_PROP_FPS, 30)
# time.sleep(0.25)

# # initialize the first frame in the video stream
# firstFrame = None
# tempFrame = None
# count = 0
# back_sub = cv2.createBackgroundSubtractorMOG2(history=2)
# kernal1 = np.ones((3,3),np.uint8)
# kernal2 = None

# # loop over the frames of the video
# while True:
#     (grabbed, frame) = camera.read()

#     if not grabbed:
#         break

#     # resize the frame, convert it to grayscale, and blur it
#     frame = cv2.resize(frame, (320,240))
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     gray = cv2.GaussianBlur(gray, (3, 3), 0)

#     # if the first frame is None, initialize it
#     if firstFrame is None:
#         print("Waiting for video to adjust...")
#         if tempFrame is None:
#             tempFrame = gray
#             continue
#         else:
#             delta = cv2.absdiff(tempFrame, gray)
#             tempFrame = gray
#             tst = cv2.threshold(delta, 5, 255, cv2.THRESH_BINARY)[1]
#             tst = cv2.dilate(tst, None, iterations=2)
#             if count > 30:
#                 print("Done.\n Waiting for motion.")
#                 if not cv2.countNonZero(tst) > 0:
#                     firstFrame = gray
#                 else:
#                     continue
#             else:
#                 count += 1
#                 continue

#     # frameDelta = cv2.absdiff(firstFrame, gray)
#     # thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
#     # thresh = cv2.dilate(thresh, None, iterations=2)
#     fgmask = back_sub.apply(frame)
#     _, fgmask = cv2.threshold(fgmask,20,255,cv2.THRESH_BINARY)
#     fgmask = cv2.erode(fgmask, kernal1, iterations=1)
#     fgmask = cv2.dilate(fgmask,kernal2,iterations=6)
#     contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#     frameCopy = frame.copy()

#     if contours is not None:
#         for c in contours:
#             if(cv2.contourArea(c) > 2000):
#                 (x, y, w, h) = cv2.boundingRect(c)
#                 cv2.rectangle(frameCopy, (x, y), (x + w, y + h), (0, 255, 0), 2)
#                 callback(c, frameCopy)
        

#     # show the frame and record if the user presses a key
#     if show_video:
#         cv2.imshow("Security Feed", frameCopy)
#         key = cv2.waitKey(1) & 0xFF

#         # if the `q` key is pressed, break from the lop
#         if key == ord("q"):
#             break

# # cleanup the camera and close any open windows
# camera.release()
# cv2.destroyAllWindows()
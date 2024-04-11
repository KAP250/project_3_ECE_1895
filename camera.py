import cv2

video_capture = cv2.VideoCapture('v4l2src device=/dev/video0 ! video/x-raw, width=(int)720, height=(int)1080 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink', cv2.CAP_GSTREAMER)
while True:
    # Capture frame-by-frame
    ret, frame = video_capture.read()

    # Display the resulting frame
    cv2.imshow('Video', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
video_capture.release()
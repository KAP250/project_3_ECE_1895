import cv2
import numpy as np

# # taking the input from webcam 
# vid = cv2.VideoCapture(0) 
  
# # running while loop just to make sure that 
# # our program keep running until we stop it 
# while True: 
#     # capturing the current frame 
#     _, frame = vid.read() 
  
#     # displaying the current frame 
#     cv2.imshow("frame", frame)  
  
#     # setting values for base colors 
#     b = frame[:, :, :1] 
#     g = frame[:, :, 1:2] 
#     r = frame[:, :, 2:] 
  
#     # computing the mean 
#     b_mean = np.mean(b) 
#     g_mean = np.mean(g) 
#     r_mean = np.mean(r) 
  
#     # displaying the most prominent color 
#     if (b_mean > g_mean and b_mean > r_mean): 
#         print("Blue") 
#     if (g_mean > r_mean and g_mean > b_mean): 
#         print("Green") 
#     else: 
#         print("Red")
cap = cv2.VideoCapture(0)
while True:
    _, frame = cap.read() 
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range of blue color in HSV
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([140, 255, 255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Apply morphological operations to remove noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((20, 20), np.uint8))

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Check if any contour is large enough
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 5000:  # Adjust this threshold as needed
            print("blue")
        else:
            print("none")
    k = cv2.waitKey(5) & 0xFF 
    if k == 27:    
        break

cv2.destroyAllWindows()
cap.release()
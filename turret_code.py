import cv2
import time
import _thread
import atexit
import sys
import termios
import contextlib
import numpy as np
import RPi.GPIO as GPIO
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper


### User Parameters ###

MOTOR_X_REVERSED = False #Base turner M1 and M2 - Motor1
MOTOR_Y_REVERSED = True #Aimer turner M3 and M4 - Motor2

LASER_PIN = 27

#######################


@contextlib.contextmanager
def raw_mode(file):
    #function that allows key presses
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)

class VideoUtils(object):
    frame = None

    @staticmethod
    def live_video(camera_port=0):
        start_point = (380, 215)
        end_point = (390,215)
        start_point2 = (385, 210)
        end_point2 = (385,220)
        color = (0, 255, 0)
        thickness = 1
        video_capture = cv2.VideoCapture(0)
        while True:
            # Capture frame-by-frame
            ret, frame = video_capture.read()
            VideoUtils.frame = frame
            frame = cv2.line(frame, start_point, end_point, color, thickness)
            frame = cv2.line(frame, start_point2, end_point2, color, thickness)
            # Display the resulting frame
            cv2.imshow('Video', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything is done, release the capture
        video_capture.release()
        cv2.destroyAllWindows()

    @staticmethod
    def color_check(frame):
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
            if area > 500:  # Adjust this threshold as needed
                print("blue")
                return True
            else:
                print("none")
                return False


    #Scrapping motion detection, Can't work if camera shakes too much, adding other changes
    # @staticmethod
    # def find_motion(callback, camera_port=0, show_video=False):
    #     camera = cv2.VideoCapture(camera_port)
    #     time.sleep(0.25)

    #     # initialize the first frame in the video stream
    #     firstFrame = None
    #     tempFrame = None
    #     count = 0

    #     # loop over the frames of the video
    #     while True:
    #         (grabbed, frame) = camera.read()

    #         if not grabbed:
    #             break

    #         # resize the frame, convert it to grayscale, and blur it
    #         frame = imutils.resize(frame, width=500)
    #         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #         gray = cv2.GaussianBlur(gray, (21, 21), 0)

    #         # if the first frame is None, initialize it
    #         if firstFrame is None:
    #             print ("Waiting for video to adjust...")
    #             if tempFrame is None:
    #                 tempFrame = gray
    #                 continue
    #             else:
    #                 delta = cv2.absdiff(tempFrame, gray)
    #                 tempFrame = gray
    #                 tst = cv2.threshold(delta, 5, 255, cv2.THRESH_BINARY)[1]
    #                 tst = cv2.dilate(tst, None, iterations=2)
    #                 if count > 30:
    #                     print ("Done.\n Waiting for motion.")
    #                     if not cv2.countNonZero(tst) > 0:
    #                         firstFrame = gray
    #                     else:
    #                         continue
    #                 else:
    #                     count += 1
    #                     continue

    #         frameDelta = cv2.absdiff(firstFrame, gray)
    #         thresh = cv2.threshold(frameDelta, 50, 255, cv2.THRESH_BINARY)[1]
    #         thresh = cv2.dilate(thresh, None, iterations=2)
    #         c = VideoUtils.get_best_contour(thresh.copy(), 5000)

    #         if c is not None:
    #             (x, y, w, h) = cv2.boundingRect(c)
    #             cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    #             callback(c, frame)

    #         if show_video:
    #             cv2.imshow("Security Feed", frame)
    #             key = cv2.waitKey(1) & 0xFF
    #             if key == ord("q"):
    #                 break

    #     camera.release()
    #     cv2.destroyAllWindows()
    # @staticmethod
    # def get_best_contour(imgmask, threshold):
    #     contours, hierarchy = cv2.findContours(imgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #     best_area = threshold
    #     best_cnt = None
    #     for cnt in contours:
    #         area = cv2.contourArea(cnt)
    #         if area > best_area:
    #             best_area = area
    #             best_cnt = cnt
    #     return best_cnt
    

class Turret(object):
    def __init__(self, friendly_mode=True):
        self.friendly_mode = friendly_mode

        #Create to control motors
        self.kit = MotorKit()
        atexit.register(self.__turn_off_motors)

        self.current_x_steps = 0
        self.current_y_steps = 0

        # Laser
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LASER_PIN, GPIO.OUT)
        GPIO.output(LASER_PIN, GPIO.LOW)

    def calibrate(self):
        print ("Please calibrate the tilt of the gun so that it is level. Commands: (w) moves up, (s) moves down. Press (enter) to finish.\n")
        self.__calibrate_y_axis()

        print ("Please calibrate the yaw of the gun so that it aligns with the camera. Commands: (a) moves left, (d) moves right. Press (enter) to finish.\n")
        self.__calibrate_x_axis()

        print ("Calibration finished.")

    def __calibrate_x_axis(self):
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch:
                        break

                    elif ch == "a":
                        if MOTOR_X_REVERSED:
                            Turret.move_backward(self, "1", 10)
                        else:
                            Turret.move_forward(self, "1", 10)
                    elif ch == "d":
                        if MOTOR_X_REVERSED:
                            Turret.move_forward(self, "1", 10)
                        else:
                            Turret.move_backward(self, "1", 10)
                    elif ch == "\n":
                        break

            except (KeyboardInterrupt, EOFError):
                print ("Error: Unable to calibrate turret. Exiting...")
                sys.exit(1)

    def __calibrate_y_axis(self):
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch:
                        break

                    if ch == "w":
                        if MOTOR_Y_REVERSED:
                            Turret.move_forward(self, "2", 10)
                        else:
                            Turret.move_backward(self, "2", 10)
                    elif ch == "s":
                        if MOTOR_Y_REVERSED:
                            Turret.move_backward(self, "2", 10)
                        else:
                            Turret.move_forward(self, "2", 10)
                    elif ch == "\n":
                        break

            except (KeyboardInterrupt, EOFError):
                print ("Error: Unable to calibrate turret. Exiting...")
                sys.exit(1)

    #Scrapping motion detection, Can't work if camera shakes too much, adding other changes
    # def motion_detection(self, show_video=False):
    #     VideoUtils.find_motion(self.__move_axis, show_video=show_video)
    # def __move_axis(self, contour, frame):
    #     (v_h, v_w) = frame.shape[:2]
    #     (x, y, w, h) = cv2.boundingRect(contour)

    #     # find height
    #     target_steps_x = (2*MAX_STEPS_X * (x + w / 2) / v_w) - MAX_STEPS_X
    #     #target_steps_y = (2*MAX_STEPS_Y*(y+h/2) / v_h) - MAX_STEPS_Y

    #     t_x = threading.Thread()
    #     #t_y = threading.Thread()
    #     #t_fire = threading.Thread()

    #     # move x
    #     if (target_steps_x - self.current_x_steps) > 0:
    #         self.current_x_steps += 1
    #         if MOTOR_X_REVERSED:
    #             t_x = threading.Thread(target=Turret.move_forward, args=(self, "1", 10))
    #         else:
    #             t_x = threading.Thread(target=Turret.move_backward, args=(self, "1", 10))
    #     elif (target_steps_x - self.current_x_steps) < 0:
    #         self.current_x_steps -= 1
    #         if MOTOR_X_REVERSED:
    #             t_x = threading.Thread(target=Turret.move_backward, args=(self, "1", 10))
    #         else:
    #             t_x = threading.Thread(target=Turret.move_forward, args=(self, "1", 10))

    #     # move y
    #     # if (target_steps_y - self.current_y_steps) > 0:
    #     #     self.current_y_steps += 1
    #     #     if MOTOR_Y_REVERSED:
    #     #         t_y = threading.Thread(target=Turret.move_backward, args=(self, "2", 2))
    #     #     else:
    #     #         t_y = threading.Thread(target=Turret.move_forward, args=(self, "2", 2))
    #     # elif (target_steps_y - self.current_y_steps) < 0:
    #     #     self.current_y_steps -= 1
    #     #     if MOTOR_Y_REVERSED:
    #     #         t_y = threading.Thread(target=Turret.move_forward, args=(self, "2", 2))
    #     #     else:
    #     #         t_y = threading.Thread(target=Turret.move_backward, args=(self, "2", 2))

    #     # fire if necessary
    #     # if not self.friendly_mode:
    #     #     if abs(target_steps_y - self.current_y_steps) <= 2 and abs(target_steps_x - self.current_x_steps) <= 2:
    #     #         t_fire = threading.Thread(target=Turret.fire)

    #     t_x.start()
    #     #t_y.start()
    #     # t_fire.start()
    #     t_x.join()
    #     #t_y.join()
    #     # t_fire.join()


    def interactive(self):
        print ('Commands: Pivot with (a) and (d). Tilt with (w) and (s). Shoot with Enter. Exit with (q)\n')
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch or ch == "q":
                        break

                    if ch == "w":
                        if MOTOR_Y_REVERSED:
                            Turret.move_forward(self, "2", 5)
                        else:
                            Turret.move_backward(self, "2", 5)
                    elif ch == "s":
                        if MOTOR_Y_REVERSED:
                            Turret.move_backward(self, "2", 5)
                        else:
                            Turret.move_forward(self, "2", 5)
                    elif ch == "a":
                        if MOTOR_X_REVERSED:
                            Turret.move_backward(self, "1", 5)
                        else:
                            Turret.move_forward(self, "1", 5)
                    elif ch == "d":
                        if MOTOR_X_REVERSED:
                            Turret.move_forward(self, "1", 5)
                        else:
                            Turret.move_backward(self, "1", 5)
                    elif ch == "\n":
                        Turret.fire()

            except (KeyboardInterrupt, EOFError):
                pass

    @staticmethod
    def fire():
        blues = 0
        nones = 0
        for i in range(100):
            boolVal = VideoUtils.color_check(VideoUtils.frame)
            if(boolVal == True):
                blues += 1
            else:
                nones += 1
        if(blues > 20):
            print("Friendly detected, not shooting")
        else:
            GPIO.output(LASER_PIN, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(LASER_PIN, GPIO.LOW)
            print("Enemy shot!")
        return

    @staticmethod
    def move_forward(self, motor, steps):
        if(motor == "1"):
            for i in range(steps):
                self.kit.stepper1.onestep(direction=stepper.FORWARD, style=stepper.MICROSTEP)
        elif(motor == "2"):
            for i in range(steps):
                self.kit.stepper2.onestep(direction=stepper.FORWARD, style=stepper.MICROSTEP)

    @staticmethod
    def move_backward(self, motor, steps):
        if(motor == "1"):
            for i in range(steps):
                self.kit.stepper1.onestep(direction=stepper.BACKWARD, style=stepper.MICROSTEP)
        elif(motor == "2"):
            for i in range(steps):
                self.kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.MICROSTEP)

    def __turn_off_motors(self):
        self.kit.stepper1.release()
        self.kit.stepper2.release()
        

if __name__ == "__main__":
    t = Turret(friendly_mode=False)

    if input("Live video? (y, n)\n").lower() == "y":
        _thread.start_new_thread(VideoUtils.live_video, ())
    t.calibrate()
    t.interactive()
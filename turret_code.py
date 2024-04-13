import cv2
import time
import _thread
import threading
import atexit
import sys
import termios
import contextlib
import imutils
import RPi.GPIO as GPIO
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper


### User Parameters ###

MOTOR_X_REVERSED = True #Base turner M1 and M2 - Motor1
MOTOR_Y_REVERSED = True #Aimer turner M3 and M4 - Motor2

MAX_STEPS_X = 30
MAX_STEPS_Y = 30

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
    @staticmethod
    def live_video(camera_port=0):
        """
        Opens a window with live video.
        """
        video_capture = cv2.VideoCapture(0)
        while True:
            # Capture frame-by-frame
            ret, frame = video_capture.read()

            # Display the resulting frame
            cv2.imshow('Video', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything is done, release the capture
        video_capture.release()
        cv2.destroyAllWindows()

    @staticmethod
    def find_motion(callback, camera_port=0, show_video=False):
        camera = cv2.VideoCapture(camera_port)
        time.sleep(0.25)

        # initialize the first frame in the video stream
        firstFrame = None
        tempFrame = None
        count = 0

        # loop over the frames of the video
        while True:
            # grab the current frame and initialize the occupied/unoccupied
            # text

            (grabbed, frame) = camera.read()

            # if the frame could not be grabbed, then we have reached the end
            # of the video
            if not grabbed:
                break

            # resize the frame, convert it to grayscale, and blur it
            frame = imutils.resize(frame, width=500)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)

            # if the first frame is None, initialize it
            if firstFrame is None:
                print("Waiting for video to adjust...")
                if tempFrame is None:
                    tempFrame = gray
                    continue
                else:
                    delta = cv2.absdiff(tempFrame, gray)
                    tempFrame = gray
                    tst = cv2.threshold(delta, 5, 255, cv2.THRESH_BINARY)[1]
                    tst = cv2.dilate(tst, None, iterations=2)
                    if count > 30:
                        print("Done.\n Waiting for motion.")
                        if not cv2.countNonZero(tst) > 0:
                            firstFrame = gray
                        else:
                            continue
                    else:
                        count += 1
                        continue

            # compute the absolute difference between the current frame and
            # first frame
            frameDelta = cv2.absdiff(firstFrame, gray)
            thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

            # dilate the thresholded image to fill in holes, then find contours
            # on thresholded image
            thresh = cv2.dilate(thresh, None, iterations=2)
            c = VideoUtils.get_best_contour(thresh.copy(), 5000)

            if c is not None:
                # compute the bounding box for the contour, draw it on the frame,
                # and update the text
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                callback(c, frame)

            # show the frame and record if the user presses a key
            if show_video:
                cv2.imshow("Security Feed", frame)
                key = cv2.waitKey(1) & 0xFF

                # if the `q` key is pressed, break from the lop
                if key == ord("q"):
                    break

        # cleanup the camera and close any open windows
        camera.release()
        cv2.destroyAllWindows()


    #     camera = cv2.VideoCapture(camera_port)
    #     time.sleep(.25)

    #     # Create background subtractor
    #     bg_subtractor = cv2.createBackgroundSubtractorMOG2()

    #     # Initialize variables
    #     last_motion_time = time.time()
    #     motion_delay = 1.0  # Minimum delay between detecting motions (in seconds)

    #     while True:
    #         grabbed, frame = camera.read()
    #         if not grabbed:
    #             break

    #         # Apply background subtraction
    #         fg_mask = bg_subtractor.apply(frame)

    #         # Process the foreground mask (e.g., find contours)
    #         contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #         for contour in contours:
    #             # Check if the contour has valid points
    #             if len(contour) >= 5:
    #                 # Calculate bounding rectangle for each contour
    #                 x, y, w, h = cv2.boundingRect(contour)

    #                 # Skip invalid contours (e.g., small areas)
    #                 if w > 0 and h > 0:
    #                     # Draw the rectangle on the original frame
    #                     cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    #                     # Pass the bounding box coordinates to the callback function
    #                     callback((x, y, w, h), frame)

    #                     # Update last motion time
    #                     last_motion_time = time.time()

    #         # Display the result
    #         if show_video:
    #             cv2.imshow("Motion Detection", frame)

    #         # Check if enough time has passed since the last motion
    #         if time.time() - last_motion_time >= motion_delay:
    #             key = cv2.waitKey(1) & 0xFF
    #             if key == ord("q"):
    #                 break

    #     camera.release()
    #     cv2.destroyAllWindows()

    @staticmethod
    def get_best_contour(imgmask, threshold):
        contours, hierarchy = cv2.findContours(imgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = threshold
        best_cnt = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > best_area:
                best_area = area
                best_cnt = cnt
        return best_cnt


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
                            Turret.move_backward(self, "1", 5)
                        else:
                            Turret.move_forward(self, "1", 5)
                    elif ch == "d":
                        if MOTOR_X_REVERSED:
                            Turret.move_forward(self, "1", 5)
                        else:
                            Turret.move_backward(self, "1", 5)
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
                            Turret.move_forward(self, "2", 5)
                        else:
                            Turret.move_backward(self, "2", 5)
                    elif ch == "s":
                        if MOTOR_Y_REVERSED:
                            Turret.move_backward(self, "2", 5)
                        else:
                            Turret.move_forward(self, "2", 5)
                    elif ch == "\n":
                        break

            except (KeyboardInterrupt, EOFError):
                print ("Error: Unable to calibrate turret. Exiting...")
                sys.exit(1)

    def motion_detection(self, show_video=False):
        VideoUtils.find_motion(self.__move_axis, show_video=show_video)

    def __move_axis(self, contour, frame):
        (v_h, v_w) = frame.shape[:2]
        (x, y, w, h) = cv2.boundingRect(contour)

        # find height
        target_steps_x = (2*MAX_STEPS_X * (x + w / 2) / v_w) - MAX_STEPS_X
        target_steps_y = (2*MAX_STEPS_Y*(y+h/2) / v_h) - MAX_STEPS_Y

        #print ("x: %s, y: %s" % (str(target_steps_x), str(target_steps_y)))
        #print ("current x: %s, current y: %s" % (str(self.current_x_steps), str(self.current_y_steps)))

        t_x = threading.Thread()
        t_y = threading.Thread()
        t_fire = threading.Thread()

        # move x
        if (target_steps_x - self.current_x_steps) > 0:
            self.current_x_steps += 1
            if MOTOR_X_REVERSED:
                t_x = threading.Thread(target=Turret.move_forward, args=(self, "1", 2))
            else:
                t_x = threading.Thread(target=Turret.move_backward, args=(self, "1", 2))
        elif (target_steps_x - self.current_x_steps) < 0:
            self.current_x_steps -= 1
            if MOTOR_X_REVERSED:
                t_x = threading.Thread(target=Turret.move_backward, args=(self, "1", 2))
            else:
                t_x = threading.Thread(target=Turret.move_forward, args=(self, "1", 2))

        # move y
        if (target_steps_y - self.current_y_steps) > 0:
            self.current_y_steps += 1
            if MOTOR_Y_REVERSED:
                t_y = threading.Thread(target=Turret.move_backward, args=(self, "2", 2))
            else:
                t_y = threading.Thread(target=Turret.move_forward, args=(self, "2", 2))
        elif (target_steps_y - self.current_y_steps) < 0:
            self.current_y_steps -= 1
            if MOTOR_Y_REVERSED:
                t_y = threading.Thread(target=Turret.move_forward, args=(self, "2", 2))
            else:
                t_y = threading.Thread(target=Turret.move_backward, args=(self, "2", 2))

        # fire if necessary
        if not self.friendly_mode:
            if abs(target_steps_y - self.current_y_steps) <= 2 and abs(target_steps_x - self.current_x_steps) <= 2:
                t_fire = threading.Thread(target=Turret.fire)

        t_x.start()
        t_y.start()
        t_fire.start()

        t_x.join()
        t_y.join()
        t_fire.join()

    def interactive(self):
        """
        Starts an interactive session. Key presses determine movement.
        :return:
        """

        Turret.move_forward(self, "1", 1)
        Turret.move_forward(self, "2", 1)

        print ('Commands: Pivot with (a) and (d). Tilt with (w) and (s). Exit with (q)\n')
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch or ch == "q":
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
                        Turret.fire()

            except (KeyboardInterrupt, EOFError):
                pass

    @staticmethod
    def fire():
        GPIO.output(LASER_PIN, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(LASER_PIN, GPIO.LOW)
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
        """
        Moves the stepper motor backward the specified number of steps
        :param motor:
        :param steps:
        :return:
        """
        if(motor == "1"):
            for i in range(steps):
                self.kit.stepper1.onestep(direction=stepper.BACKWARD, style=stepper.MICROSTEP)
        elif(motor == "2"):
            for i in range(steps):
                self.kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.MICROSTEP)

    def __turn_off_motors(self):
        """
        Recommended for auto-disabling motors on shutdown!
        :return:
        """
        self.kit.stepper1.release()
        self.kit.stepper2.release()
        

if __name__ == "__main__":
    t = Turret(friendly_mode=False)

    user_input = input("Choose an input mode: (1) Motion Detection, (2) Interactive\n")

    if user_input == "1":
        t.calibrate()
        if input("Live video? (y, n)\n").lower() == "y":
            t.motion_detection(show_video=True)
        else:
            t.motion_detection()
    elif user_input == "2":
        if input("Live video? (y, n)\n").lower() == "y":
            _thread.start_new_thread(VideoUtils.live_video, ())
        t.interactive()
    else:
        print ("Unknown input mode. Please choose a number (1) or (2)")
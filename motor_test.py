import time
import RPi.GPIO as GPIO
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper

#Motor 1
#Config is black and green on M1(white to green, red to black)
#Config is blue and red on M2(orange to blue, purple to red)
#Is reverse so .BACKWARD goes CW and .FORWARD goes CCW

#Motor 2
#Config is black and green on M3(green to black, red to green)
#Config is blue and red on M4(grey to blue, white to red)
#Is NOT reverse so .BACKWARD goes CCW and .FORWARD goes CW
kit = MotorKit()
MOTOR_X_REVERSED = True #Base turner M1 and M2 - Motor1
MOTOR_Y_REVERSED = False #Aimer turner M3 and M4 - Motor2

#Motor 2 test
# for i in range(20):
#     kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.MICROSTEP)

#Motor 1 test
for i in range(200):
    kit.stepper1.onestep(direction=stepper.FORWARD, style=stepper.MICROSTEP)

#Laser testing
LASER_PIN = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(LASER_PIN, GPIO.OUT)
GPIO.output(LASER_PIN, GPIO.LOW)

GPIO.output(LASER_PIN, GPIO.HIGH)
time.sleep(1)
GPIO.output(LASER_PIN, GPIO.LOW)

kit.stepper1.release()
kit.stepper2.release()

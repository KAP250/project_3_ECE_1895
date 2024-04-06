import cv2
import RPi._GPIO as GPIO
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
print(cv2.__version__)

MOTOR_X_REVERSED = False
MOTOR_Y_REVERSED = False

MAX_STEPS_X = 30
MAX_STEPS_Y = 15

RELAY_PIN = 22


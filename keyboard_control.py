# import keyboard
import RPi.GPIO as GPIO
from time import sleep
import numpy as np
import Encoder
import threading
import signal
import sys
import asyncio

# For GPIO clean exit
def signal_handler(sig, frame):
    print('Cleaning GPIO and Exiting the program...')
    exitRoutine()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Motor--------------------------------------
pwm_frequency = 1000 
encoder_count_per_rotation = 810    
V = 12

# GPIOs--------------------------------------
# First Motor related
motor_driver_1_reverse_enable_pin = 6       # GPIO 4
motor_driver_1_forward_enable_pin = 13      # GPIO 17
motor_driver_1_reverse_pwm_pin = 19         # GPIO 27
motor_driver_1_forward_pwm_pin = 26         # GPIO 22
motor_1_Encoder_A_pin = 12                  # GPIO 18
motor_1_Encoder_B_pin = 16                  # GPIO 23

# Second Motor related
motor_driver_2_reverse_enable_pin = 10      # GPIO 10
motor_driver_2_forward_enable_pin = 9       # GPIO 9
motor_driver_2_reverse_pwm_pin = 11         # GPIO 11
motor_driver_2_forward_pwm_pin = 5          # GPIO 5
motor_2_Encoder_A_pin = 24                  # GPIO 24
motor_2_Encoder_B_pin = 25                  # GPIO 25

# Third Motor related
motor_driver_3_reverse_enable_pin = 4     # GPIO 6
motor_driver_3_forward_enable_pin = 17      # GPIO 13
motor_driver_3_reverse_pwm_pin = 27         # GPIO 19
motor_driver_3_forward_pwm_pin = 22         # GPIO 26
motor_3_Encoder_A_pin = 18                  # GPIO 12
motor_3_Encoder_B_pin = 23                  # GPIO 16

# GPIO initialization--------------------------------------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
# First Motor related
GPIO.setup(motor_driver_1_reverse_enable_pin, GPIO.OUT)
GPIO.setup(motor_driver_1_forward_enable_pin, GPIO.OUT)
GPIO.setup(motor_driver_1_reverse_pwm_pin, GPIO.OUT)
GPIO.setup(motor_driver_1_forward_pwm_pin, GPIO.OUT)
GPIO.setup(motor_1_Encoder_A_pin, GPIO.IN)
GPIO.setup(motor_1_Encoder_B_pin, GPIO.IN)
motor_1_encoder = Encoder.Encoder(motor_1_Encoder_A_pin, motor_1_Encoder_B_pin)

motor_driver_1_reverse_pwm = GPIO.PWM(motor_driver_1_reverse_pwm_pin, pwm_frequency)
motor_driver_1_forward_pwm = GPIO.PWM(motor_driver_1_forward_pwm_pin, pwm_frequency)

# Second Motor related
GPIO.setup(motor_driver_2_reverse_enable_pin, GPIO.OUT)
GPIO.setup(motor_driver_2_forward_enable_pin, GPIO.OUT)
GPIO.setup(motor_driver_2_reverse_pwm_pin, GPIO.OUT)
GPIO.setup(motor_driver_2_forward_pwm_pin, GPIO.OUT)
GPIO.setup(motor_2_Encoder_A_pin, GPIO.IN)
GPIO.setup(motor_2_Encoder_B_pin, GPIO.IN)
motor_2_encoder = Encoder.Encoder(motor_2_Encoder_A_pin, motor_2_Encoder_B_pin)

motor_driver_2_reverse_pwm = GPIO.PWM(motor_driver_2_reverse_pwm_pin, pwm_frequency)
motor_driver_2_forward_pwm = GPIO.PWM(motor_driver_2_forward_pwm_pin, pwm_frequency)

# Third Motor related
GPIO.setup(motor_driver_3_reverse_enable_pin, GPIO.OUT)
GPIO.setup(motor_driver_3_forward_enable_pin, GPIO.OUT)
GPIO.setup(motor_driver_3_reverse_pwm_pin, GPIO.OUT)
GPIO.setup(motor_driver_3_forward_pwm_pin, GPIO.OUT)
GPIO.setup(motor_3_Encoder_A_pin, GPIO.IN)
GPIO.setup(motor_3_Encoder_B_pin, GPIO.IN)
motor_3_encoder = Encoder.Encoder(motor_3_Encoder_A_pin, motor_3_Encoder_B_pin)

motor_driver_3_reverse_pwm = GPIO.PWM(motor_driver_3_reverse_pwm_pin, pwm_frequency)
motor_driver_3_forward_pwm = GPIO.PWM(motor_driver_3_forward_pwm_pin, pwm_frequency)
# End of initialization--------------------------------------

def rotateCCW(motor, voltage):
    global motor_driver_1_forward_pwm
    global motor_driver_2_forward_pwm
    global motor_driver_3_forward_pwm
    global V

    pwm_percent = voltage / V * 100
    if(motor == 0):
        motor_driver_1_forward_pwm.ChangeDutyCycle(pwm_percent)
    elif (motor == 1):
        motor_driver_2_forward_pwm.ChangeDutyCycle(pwm_percent)
    elif (motor == 2):
        motor_driver_3_forward_pwm.ChangeDutyCycle(pwm_percent)

def rotateCW(motor, voltage):
    global motor_driver_1_reverse_pwm
    global motor_driver_2_reverse_pwm
    global motor_driver_3_reverse_pwm
    global V

    pwm_percent = voltage / V * 100
    if(motor == 0):
        motor_driver_1_reverse_pwm.ChangeDutyCycle(pwm_percent)
    elif (motor == 1):
        motor_driver_2_reverse_pwm.ChangeDutyCycle(pwm_percent)
    elif (motor == 2):
        motor_driver_3_reverse_pwm.ChangeDutyCycle(pwm_percent)

def stopRotate(motor):
    rotateCW(motor, 0)
    rotateCCW(motor, 0)

def getEncoderPosition(encoder):
    global motor_1_encoder
    global motor_2_encoder
    global motor_3_encoder
    global encoder_count_per_rotation

    if(encoder == 0):
        return 2* np.pi * (motor_1_encoder.read() / 10) / (encoder_count_per_rotation)  # rad
    elif (encoder == 1):
        return 2* np.pi * (motor_2_encoder.read() / 10) / (encoder_count_per_rotation)  # rad
    elif (encoder == 2):
        return 2* np.pi * (motor_3_encoder.read() / 10) / (encoder_count_per_rotation)  # rad

def getEncoderVelocity(encoder_position, prev_pos, dt):
    return (encoder_position - prev_pos) / (dt) # rad/s

def exitRoutine():
    GPIO.cleanup() 


dt = 0.05 #50ms
prev_pos = 0
prev_pos1 = 1
prev_pos2 = 2

GPIO.output(motor_driver_1_reverse_enable_pin, GPIO.HIGH)
GPIO.output(motor_driver_1_forward_enable_pin, GPIO.HIGH)

GPIO.output(motor_driver_2_reverse_enable_pin, GPIO.HIGH)
GPIO.output(motor_driver_2_forward_enable_pin, GPIO.HIGH)

GPIO.output(motor_driver_3_reverse_enable_pin, GPIO.HIGH)
GPIO.output(motor_driver_3_forward_enable_pin, GPIO.HIGH)

motor_driver_1_forward_pwm.start(0)
motor_driver_1_reverse_pwm.start(0)

motor_driver_2_forward_pwm.start(0)
motor_driver_2_reverse_pwm.start(0)

motor_driver_3_forward_pwm.start(0)
motor_driver_3_reverse_pwm.start(0)

# Keyboard Listeners---------------------
a_state = 0
d_state = 0
w_state = 0
s_state = 0
r_state = 0
f_state = 0
# a ------------------------------------
def on_press_a(key):
    global a_state
    if(a_state == 0):
        rotateCW(0, 4)
    # a_state = 1

def on_release_a(key):
    global a_state
    stopRotate(0)
    a_state = 0

# d ------------------------------------
def on_press_d(key):
    global d_state

    if(d_state == 0):
        rotateCCW(0, 4)

    # d_state = 1

def on_release_d(key):
    global d_state
    stopRotate(0)
    d_state = 0

# w ------------------------------------
def on_press_w(key):
    global w_state

    if(w_state == 0):
        rotateCCW(1, 8)

    # w_state = 1

def on_release_w(key):
    global w_state
    stopRotate(1)
    w_state = 0

# s ------------------------------------
def on_press_s(key):
    global s_state

    if(s_state == 0):
        rotateCW(1, 8)

    # s_state = 1

def on_release_s(key):
    global s_state
    stopRotate(1)
    s_state = 0

# r ------------------------------------
def on_press_r(key):
    global r_state

    if(r_state == 0):
        rotateCW(2, 6)
        
    # print("pressed rrrrrr")
    # r_state = 1

def on_release_r(key):
    global r_state
    stopRotate(2)
    r_state = 0

# f ------------------------------------
def on_press_f(key):
    global f_state

    if(f_state == 0):
        rotateCCW(2, 6)

    # f_state = 1

def on_release_f(key):
    global f_state
    stopRotate(2)
    f_state = 0

# keyboard.on_press_key("a", on_press_a)
# keyboard.on_release_key("a", on_release_a)

# keyboard.on_press_key("d", on_press_d)
# keyboard.on_release_key("d", on_release_d)

# keyboard.on_press_key("w", on_press_w)
# keyboard.on_release_key("w", on_release_w)

# keyboard.on_press_key("s", on_press_s)
# keyboard.on_release_key("s", on_release_s)

# keyboard.on_press_key("r", on_press_r)
# keyboard.on_release_key("r", on_release_r)

# keyboard.on_press_key("f", on_press_f)
# keyboard.on_release_key("f", on_release_f)

async def main():
    await asyncio.sleep(dt)
    global prev_pos
    global prev_pos1
    global prev_pos2


    pos0 = getEncoderPosition(0)
    vel0 = getEncoderVelocity(pos0, prev_pos, dt)

    pos1 = getEncoderPosition(1)
    vel1 = getEncoderVelocity(pos1, prev_pos1, dt)

    pos2 = getEncoderPosition(2)
    vel2 = getEncoderVelocity(pos2, prev_pos2, dt)

    print("position: " + str(pos0) + ". velocity: " + str(vel0) + ".")
    print("position1: " + str(pos1) + ". velocity: " + str(vel1) + ".")
    print("position2: " + str(pos2) + ". velocity: " + str(vel2) + ".")
    print("---------------------------------------------------------")

    prev_pos = pos0
    prev_pos1 = pos1
    prev_pos2 = pos2
    
    #threading.Timer(dt, main).start()  
asyncio.run(main())

while True:
    keyboard = input("W S A D R F space: ")
    if keyboard == "w":
        on_press_r(keyboard)
    elif keyboard == "s":
        on_press_r(keyboard)
    elif keyboard == "a":
        on_press_r(keyboard)
    elif keyboard == "d":
        on_press_r(keyboard)
    elif keyboard == "r":
        on_press_r(keyboard)
    elif keyboard == "f":
        on_press_r(keyboard)
    elif keyboard == " ":
        stopRotate(0)
        stopRotate(1)
        stopRotate(2)


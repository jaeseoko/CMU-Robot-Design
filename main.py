import RPi.GPIO as GPIO
from time import sleep

# Motor--------------------------------------
pwm_frequency = 1000    

# GPIOs--------------------------------------
# First Motor related
motor_driver_1_reverse_enable_pin = 4       # GPIO 4
motor_driver_1_forward_enable_pin = 17      # GPIO 17
motor_driver_1_reverse_pwm_pin = 27         # GPIO 27
motor_driver_1_forward_pwm_pin = 22         # GPIO 22
motor_1_Encoder_A_pin = 18                  # GPIO 18
motor_1_Encoder_B_pin = 23                  # GPIO 23

# Second Motor related
motor_driver_2_reverse_enable_pin = 10      # GPIO 10
motor_driver_2_forward_enable_pin = 9       # GPIO 9
motor_driver_2_reverse_pwm_pin = 11         # GPIO 11
motor_driver_2_forward_pwm_pin = 5          # GPIO 5
motor_2_Encoder_A_pin = 24                  # GPIO 24
motor_2_Encoder_B_pin = 25                  # GPIO 25

# Third Motor related
motor_driver_3_reverse_enable_pin = 6       # GPIO 6
motor_driver_3_forward_enable_pin = 13      # GPIO 13
motor_driver_3_reverse_pwm_pin = 19         # GPIO 19
motor_driver_3_forward_pwm_pin = 26         # GPIO 26
motor_3_Encoder_A_pin = 12                  # GPIO 12
motor_3_Encoder_B_pin = 16                  # GPIO 16

# GPIO initialization--------------------------------------
GPIO.setmode(GPIO.BCM)
# First Motor related
GPIO.setup(motor_driver_1_reverse_enable_pin, GPIO.OUT)
GPIO.setup(motor_driver_1_forward_enable_pin, GPIO.OUT)
GPIO.setup(motor_driver_1_reverse_pwm_pin, GPIO.OUT)
GPIO.setup(motor_driver_1_forward_pwm_pin, GPIO.OUT)
GPIO.setup(motor_1_Encoder_A_pin, GPIO.IN)
GPIO.setup(motor_1_Encoder_B_pin, GPIO.IN)

motor_driver_1_reverse_pwm = GPIO.PWM(motor_driver_1_reverse_pwm_pin, pwm_frequency)
motor_driver_1_forward_pwm = GPIO.PWM(motor_driver_1_reverse_pwm_pin, pwm_frequency)

# Second Motor related
GPIO.setup(motor_driver_2_reverse_enable_pin, GPIO.OUT)
GPIO.setup(motor_driver_2_forward_enabl_pine, GPIO.OUT)
GPIO.setup(motor_driver_2_reverse_pwm_pin, GPIO.OUT)
GPIO.setup(motor_driver_2_forward_pwm_pin, GPIO.OUT)
GPIO.setup(motor_2_Encoder_A_pin, GPIO.IN)
GPIO.setup(motor_2_Encoder_B_pin, GPIO.IN)

motor_driver_2_reverse_pwm = GPIO.PWM(motor_driver_2_reverse_pwm_pin, pwm_frequency)
motor_driver_2_forward_pwm = GPIO.PWM(motor_driver_2_reverse_pwm_pin, pwm_frequency)

# Third Motor related
GPIO.setup(motor_driver_3_reverse_enable_pin, GPIO.OUT)
GPIO.setup(motor_driver_3_forward_enable_pin, GPIO.OUT)
GPIO.setup(motor_driver_3_reverse_pwm_pin, GPIO.OUT)
GPIO.setup(motor_driver_3_forward_pwm_pin, GPIO.OUT)
GPIO.setup(motor_3_Encoder_A_pin, GPIO.IN)
GPIO.setup(motor_3_Encoder_B_pin, GPIO.IN)

motor_driver_3_reverse_pwm = GPIO.PWM(motor_driver_3_reverse_pwm_pin, pwm_frequency)
motor_driver_3_forward_pwm = GPIO.PWM(motor_driver_3_reverse_pwm_pin, pwm_frequency)
# End of initialization--------------------------------------



startMsg = "Please specify the motor and angle \n Example: \"1 30\", or \"2 60\""
print(startMsg)


while(1):
    message = input()
    print("You have inputed: " + message)

import RPi.GPIO as GPIO          
from time import sleep

ser = serial.Serial(
        port='/dev/ttyS0',
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)


print("\n")
print("Please specify the motor and angle")
print("Example: \"1 30\" or \"2 60\"")
print("\n")    

while(1):
        message = ser.readline()
        print(message)

    
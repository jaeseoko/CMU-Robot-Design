import RPi.GPIO as GPIO          
from time import sleep

startMsg = "Please specify the motor and angle \n Example: \"1 30\", or \"2 60\""
print(startMsg)


while(1):
        message = input()
        print("You have inputed: " + message)

    

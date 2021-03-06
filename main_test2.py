import RPi.GPIO as GPIO
from time import sleep
import numpy as np
import Encoder
import threading
import signal
import sys
import pybullet as p
import argparse
import time
# import keyboard

# For pybullet loading urdf to calculate inverse dynamics / Motor Params

def saveTorque(myArray):
    
    # x = [['ABC', 123.45], ['DEF', 678.90]]
    np.savetxt('torqueLog.txt', myArray, fmt='%s')
    # default dtype for  np.loadtxt is also floating point, change it, to be able to load mixed data.
    # y = np.loadtxt('text.txt', dtype=np.object)

def SetUp():
    global ee_mass, bodyId
    client = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81, physicsClientId=client)
    flags = p.URDF_USE_SELF_COLLISION
    bodyId = p.loadURDF("./data/Arm_Final_Planet_hook/urdf/Arm_Final_Planet_hook.urdf",
                        basePosition=[0,0,0],useFixedBase=True,flags=flags)
    maxForce = 0
    p.setJointMotorControl2(bodyId, 0,controlMode=p.VELOCITY_CONTROL, force=maxForce)
    p.setJointMotorControl2(bodyId, 1,controlMode=p.VELOCITY_CONTROL, force=maxForce)
    p.setJointMotorControl2(bodyId, 2,controlMode=p.VELOCITY_CONTROL, force=maxForce)
    # end-effector mass in bullet.
    ee_mass = p.getDynamicsInfo(bodyId,3)[0]

    parser = argparse.ArgumentParser()
    parser.add_argument('a0', 
                        type=float,
                        help='target end effector joint angle 0')
    parser.add_argument('a1', 
                        type=float,
                        help='target end effector joint angle 1')
    parser.add_argument('a2', 
                        type=float,
                        help='target end effector joint angle 2')
    parser.add_argument('--load', 
                        type=float,
                        help='weight to lift')
    parser.add_argument('--worm',
                        type=int,
                        help='set if worm gear used or not,0: planetary 1: worm gear')
    args = parser.parse_args()
    targetORN = [args.a0*np.pi/180,args.a1*np.pi/180,args.a2*np.pi/180]
    destORN = [args.a0*np.pi/180 + np.pi/2,args.a1*np.pi/180,args.a2*np.pi/180]
    prev_pos = [0,-(85)*np.pi/180,0]
    off = [0,-(85)*np.pi/180,0]
    prev_error = [0,0,0]
    cum_e = [0,0,0]
    load = args.load
    logTorque = []
    if args.worm==0:
        worm = False
    else:
        worm = True   
    picked, placed = False, False
    offset = False
    # import time

    start_time = time.time()
    # end = time.time()

    return targetORN,destORN,prev_pos,prev_error,cum_e,load,picked,placed,offset,worm, off, logTorque, start_time

def checkPoint(error,vel,status):
    tol = 0.1
    if( status == False and np.linalg.norm(np.asarray(error),axis=0) < tol and
                            np.linalg.norm(np.asarray(vel),axis=0)   < tol):
        status = True

    return status

def GetVoltage(torque,vel):
    Ts = 23.5/1000                      # Nm (stall torque)
    Is = 1.8                            # A  (stall current)
    R = 8.4                             # Ohm
    V = 12                              # Voltage [V]
    noLoadCurr = 70/1000                # A
    noLoadSpeed = 7000*2*np.pi/60       # rad / s
    N = 270
    Kt = Ts/Is
    Ke = (V - R*noLoadCurr)/noLoadSpeed
    
    V0 = R/Kt*torque[0]/N +Ke*vel[0]*N
    V1 = R/Kt*torque[1]/N +Ke*vel[1]*N
    V2 = R/Kt*torque[2]/N +Ke*vel[2]*N

    return [V0,V1,V2]

def GetVoltageWorm(torque,vel):

    # PLANETARY for first and second
    Ts = 23.5/1000                      # Nm (stall torque)
    Is = 1.8                            # A  (stall current)
    R = 8.4                             # Ohm
    V = 12                              # Voltage [V]
    noLoadCurr = 70/1000                # A
    noLoadSpeed = 7000*2*np.pi/60       # rad / s
    N = 270
    Kt = Ts/Is
    Ke = (V - R*noLoadCurr)/noLoadSpeed

    # WORM for third
    Ts2 = 70/1000                        # Nm (stall torque)
    Is2 = 5.2                            # A  (stall current)
    R2 = 8.4                             # Ohm
    V2 = 24                              # Voltage [V]
    noLoadCurr2 = 0.25                   # A
    noLoadSpeed2 = 16*2*np.pi/60         # rad / s
    N2 = 5002
    Kt2 = Ts2/Is2
    Ke2 = (V2 - R2*noLoadCurr2)/noLoadSpeed2

    V0 = R/Kt*torque[0]/N +Ke*vel[0]*N
    V1 = R/Kt*torque[1]/N +Ke*vel[1]*N
    V2 = R2/Kt2*torque[2]/N2 +Ke2*vel[2]*N2

    return [V0,V1,V2]


def PID_torque(e,de,cum_e,load):
    # kp0,ki0,kd0 = 2e-2, 1e-8 , 2e-2
    kp0,ki0,kd0 = 9e-2, 1e-8 , 9e-2
    # kp1,ki1,kd1 = 3e-2, 1e-7 , 4e-2
    kp1,ki1,kd1 = 5.7, 1e-3 , 5.9
    # kp2,ki2,kd2 = 2e-2, 1e-4 , 2e-2
    kp2,ki2,kd2 = 9e-1, 1e-3 , 9e-1


    if(load!=0):
        kp0*=(1+ 10.5*load)
        ki0*=(1+ 5**5*load)
        kd0*=(1+ 15*load)
        kp1*=(1+ 1000.6*load)
        ki1*=(1+ 5**6*load)
        kd1*=(1+ 805*load)
        kp2*=(1+ 7.025*load)
        ki2*=(1+ 7.5*load)
        kd2*=(1+ 7.025*load)

    T0 = kp0*(e[0]) + kd0*(de[0]) + ki0*cum_e[0]
    T1 = kp1*(e[1]) + kd1*(de[1]) + ki1*cum_e[1]
    T2 = kp2*(e[2]) + kd2*(de[2]) + ki2*cum_e[2]

    return [T0,T1,T2]

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
motor_driver_3_reverse_enable_pin = 4       # GPIO 6
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

    pwm_percent = 0

    if(voltage > 12):
        pwm_percent = 100
    else:
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

    pwm_percent = 0

    if(voltage > 12):
        pwm_percent = 100
    else:
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

# rotateCW(0, 12)

# pause = 0

targetORN, destORN, prev_pos, prev_error, cum_e, load, picked, placed, offset, worm, off , logTorque, start_time= SetUp()

def main():
    global targetORN, destORN, prev_pos, prev_error, cum_e, load, picked, placed, offset, worm, off

    pos = [getEncoderPosition(0) +off[0],-getEncoderPosition(1) +off[1],getEncoderPosition(2)+off[2]]

    print("checking getEncoderPosition fun: ",getEncoderPosition(0),
                                              getEncoderPosition(1),
                                              getEncoderPosition(2))
    print("--------------------------------------------")
    
    vel = [getEncoderVelocity(pos[0], prev_pos[0], dt),
           getEncoderVelocity(pos[1], prev_pos[1], dt),
           getEncoderVelocity(pos[2], prev_pos[2], dt)]

    vel[1]*=-1

    # print("--------------------------------------------")
    # if offset ==False:
    #     targetORN[2]-=10*np.pi/180
    #     offset = True

    # error = [targetORN[0]-pos[0],targetORN[1]-pos[1],targetORN[2]-pos[2] ]

    error = [targetORN[0]-pos[0],-targetORN[1]+pos[1],targetORN[2]-pos[2] ]

    print("errors: ",error[0]*180/np.pi,error[1]*180/np.pi,error[2]*180/np.pi)
    de = [error[0] - prev_error[0],error[1] - prev_error[1],error[2] - prev_error[2] ]
    cum_e+=error
    cum_e = [cum_e[0]+error[0],cum_e[1]+error[1],cum_e[2]+error[2]]
    


    if picked == False:
        pidTorques = PID_torque(error, de, cum_e, 0)
        picked = checkPoint(error, vel, picked)
        if True==checkPoint(error, vel, picked):
            picked = True
            targetORN = destORN


    if picked == True:
        pidTorques = PID_torque(error, de, cum_e, load)
        # placed = checkPoint(error, vel, placed)

    if True==checkPoint(error, vel, placed):
        placed = True
        print("Reached goal destination.")

    tau0,tau1,tau2 = p.calculateInverseDynamics(bodyId,
                                                [pos[0],pos[1],pos[2]],
                                                [vel[0],vel[1],vel[2]],
                                                [0,0,0])
    print("--------------------------------------------")
    print("from bullet, torq 1: ",tau1)
    print("--------------------------------------------")
    tau1=tau1 + 0.2*tau1
    torque = [pidTorques[0]+tau0,pidTorques[1]+tau1,pidTorques[2]+tau2]
    
    
    
    print("torques = ", torque)
    print("--------------------------------------------")

    
    if worm==True:
        volt = GetVoltageWorm(torque, vel)

        # Turning off 
        if ( abs(vel[1])+abs(vel[2]) < 0.02 ) and \
            ( (abs(error[1]+abs(error[2])) ) < 0.5 ) :
            stopRotate(2)
            GPIO.output(motor_driver_3_reverse_enable_pin, GPIO.LOW)
            GPIO.output(motor_driver_3_forward_enable_pin, GPIO.LOW)
    else:
        volt = GetVoltage(torque,vel)

    

    # volt = [0,0,0]
    print("volt = ", volt)
    print("--------------------------------------------")

    

    # if(volt[0]>0): rotateCW(0, volt[0])
    # else: rotateCCW(0, abs(volt[0]))
    # if(volt[1]<0): rotateCW(1, abs(volt[1]))
    # else: rotateCCW(1, volt[1])
    # if picked==True and worm == True:
    #     stopRotate(2)
    # elif(volt[2]<0): 
    #     rotateCW(2, abs(volt[2]))
    # else: 
    #     rotateCCW(2, volt[2])

    if(volt[0]>0): rotateCW(0, abs(volt[0]))
    else: rotateCCW(0, abs(volt[0]))
    if(volt[1]>0): rotateCW(1, abs(volt[1]))
    else: rotateCCW(1, abs(volt[1]))
    # if picked==True and worm == True:
    #     stopRotate(2)
    if(volt[2]>0): 
        rotateCW(2, abs(volt[2]))
    else: 
        rotateCCW(2, abs(volt[2]))


    print("position 0: " + str(pos[0]*180/np.pi) + ". velocity 0: " + str(vel[0]) + ".")
    print("position 1: " + str(pos[1]*180/np.pi) + ". velocity 1: " + str(vel[1]) + ".")
    print("position 2: " + str(pos[2]*180/np.pi) + ". velocity 2: " + str(vel[2]) + ".")
    print("-----------------------------------------------------------------")

    prev_pos   = pos
    prev_error = error
    now = time.time()
    
    torque.append(now-start_time)
    logTorque.append(torque)
    saveTorque(np.asarray(logTorque))



    threading.Timer(dt, main).start()  

main()

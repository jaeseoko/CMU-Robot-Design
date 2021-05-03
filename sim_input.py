import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from Arm_Planet_IK import Arm_Planet_IK as customIK
import time
import argparse
import threading


# def get_input():
#     # data = input() # Something akin to this
#     angle = float(input("input angles : "))
    
#     return [angle0,angle1,angle2]

# input_thread = threading.Thread(target=get_input)
# input_thread.start()



Ts = 23.5/1000                      # Nm (stall torque)
Is = 1.8                            # A  (stall current)
R = 8.4                             # Ohm
V = 12                              # Voltage [V]
noLoadCurr = 70/1000                # A
noLoadSpeed = 7000*2*np.pi/60       # rad / s

Kt = Ts/Is
Ke = (V - R*noLoadCurr)/noLoadSpeed

parser = argparse.ArgumentParser()
parser.add_argument('--weight',
                    type = float,
                    help = 'set payload weight to pick up')
args = parser.parse_args()


tol = 1e-2

# Can alternatively pass in p.DIRECT 
client = p.connect(p.GUI)
# client = p.connect(p.DIRECT)
p.setGravity(0, 0, -9.81, physicsClientId=client)


p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")



''' LOAD '''
flags = p.URDF_USE_SELF_COLLISION

bodyId = p.loadURDF("./data/Arm_Final_Planet_hook/urdf/Arm_Final_Planet_hook.urdf",
                    basePosition=[0,0,0],useFixedBase=True,flags=flags)
                    

''' LOAD '''

position, orientation = p.getBasePositionAndOrientation(bodyId)
numJoints = p.getNumJoints(bodyId)
print("number of joints = ",numJoints)
# for i in range(numJoints):
#     print("Joint ", i, "\n")
#     print(p.getJointInfo(bodyId,i))






torqueLog0 = []
torqueLog1 = []
torqueLog2 = []
errorLog0  = []
errorLog1 = []
errorLog2 = []
angLog0 = []
angLog1 = []
angLog2 = []
minE = []

prev_error0 = 0
prev_error1 = 0
prev_error2 = 0
cumul_e0 = 0
cumul_e1 = 0
cumul_e2 = 0

# PARAMS
kp0 = 2e-2
ki0 = 1e-8
# kd0 = 400/240
kd0 = 2e-2

kp1 = 3e-2
ki1 = 1e-7
kd1 = 4e-2

kp2 = 2e-2
ki2 = 1e-4
kd2 = 2e-2

# print("0 -> kp,ki,kd = ",kp0,ki0,kd0)
# print("1 -> kp,ki,kd = ",kp1,ki1,kd1)
# print("2 -> kp,ki,kd = ",kp2,ki2,kd2)

prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
trailDuration = 15
hasPrevPose = 0

p.enableJointForceTorqueSensor(bodyId,0)
p.enableJointForceTorqueSensor(bodyId,1)
p.enableJointForceTorqueSensor(bodyId,2)



maxForce = 0
p.setJointMotorControl2(bodyId, 0,controlMode=p.VELOCITY_CONTROL, force=maxForce)
p.setJointMotorControl2(bodyId, 1,controlMode=p.VELOCITY_CONTROL, force=maxForce)
p.setJointMotorControl2(bodyId, 2,controlMode=p.VELOCITY_CONTROL, force=maxForce)

ind,Name,tp,qInd,uInd,flags,damp, \
friction,ll,ul,maxF,maxV,linkName, \
ax,parPos,parOrn,ParInd = p.getJointInfo(bodyId,0)
# print("friction = ",friction)

p.changeDynamics(bodyId,0, rollingFriction = 1e-5, spinningFriction = 1e-4)
p.changeDynamics(bodyId,1,jointLowerLimit = -np.pi,jointUpperLimit = np.pi, rollingFriction = 1e-5,spinningFriction = 1e-2)
p.changeDynamics(bodyId,2,jointLowerLimit = -np.pi,jointUpperLimit = np.pi, rollingFriction = 1e-5,spinningFriction = 1e-2)


ee_mass = p.getDynamicsInfo(bodyId,3)[0]
print("mass of end effector : ", ee_mass)


# tarOrn = p.getQuaternionFromEuler([np.pi/2,np.pi/2,-np.pi/2])
# desOrn = p.getQuaternionFromEuler([0,0,0])


# targetORN = customIK(target)
# print("target angles from custom IK: ",targetORN*180/np.pi)
# destORN = customIK(destination)
# print("destination angles from custom IK: ",destORN*180/np.pi)



payload = args.weight

duration = 30000


p.setRealTimeSimulation(0)
t0 = time.time()
dt = 1/240
termTime = 0
targetORN = [0,0,0]

# input_thread = threading.Thread(target=get_input)
# input_thread.start()
# print("type target first joint angle: ")
# targetORN[0] = get_input()
# # print("type target second joint angle: ")
# targetORN[1] = get_input()
# # print("type target third joint angle: ")
# targetORN[2] = get_input()

# targetORN = get_input()


# targetORN = [a0,a1,a2]

for i in range(duration): 
    
    # pos, ori = p.getBasePositionAndOrientation(bodyId)


    pos = [0,0,0]
    
    q0,v0,_,_ = p.getJointState(bodyId,0)
    q1,v1,_,_ = p.getJointState(bodyId,1)
    q2,v2,_,_ = p.getJointState(bodyId,2)
    error0 = targetORN[0]-q0
    error1 = targetORN[1]-q1
    error2 = targetORN[2]-q2
    de0 = error0 - prev_error0
    de1 = error1 - prev_error1
    de2 = error2 - prev_error2

    

    pos0,vel0,RF0,torque0 = p.getJointState(bodyId,0)
    pos1,vel1,RF1,torque1 = p.getJointState(bodyId,1)
    pos2,vel2,RF2,torque2 = p.getJointState(bodyId,2)
    tau0,tau1,tau2 = p.calculateInverseDynamics(bodyId,
                                                [pos0,pos1,pos2],
                                                [vel0,vel1,vel2],
                                                [0,0,0])
    # tau0,tau1,tau2 = p.calculateInverseDynamics(bodyId,
    #                                             [pos0,pos1,pos2],
    #                                             [0,0,0],
    #                                             [0,0,0])

    T0 = kp0*(error0) + kd0*(de0/dt) + ki0*cumul_e0
    T1 = kp1*(error1) + kd1*(de1/dt) + ki1*cumul_e1
    T2 = kp2*(error2) + kd2*(de2/dt) + ki2*cumul_e2
    # print("torques 0 1 2: ",T0,",",T1,",",T2)

    prev_error0 = error0
    prev_error1 = error1
    prev_error2 = error2

    cumul_e0 += error0
    cumul_e1 += error1
    cumul_e2 += error2

    ####
    
    ####
    # tau0 = 0
    force0 = T0 + tau0
    force1 = T1 + tau1
    force2 = T2 + tau2
    
    p.setJointMotorControl2(bodyId,0,controlMode = p.TORQUE_CONTROL, force = force0)
    p.setJointMotorControl2(bodyId,1,controlMode = p.TORQUE_CONTROL, force = force1)
    p.setJointMotorControl2(bodyId,2,controlMode = p.TORQUE_CONTROL, force = force2)

    # V0 = R/Kt*force0 +Ke*v0
    # V1 = R/Kt*force1 +Ke*v1
    # V2 = R/Kt*force2 +Ke*v2
    # print("voltages: ")
    # print(V0)
    # print(V1)
    # print(V2)
    

    errorLog0.append(error0*180/np.pi)
    errorLog1.append(error1*180/np.pi)
    errorLog2.append(error2*180/np.pi)
    torqueLog0.append(force0)
    torqueLog1.append(force1)
    torqueLog2.append(force2)
    angLog0.append(q0*180/np.pi)
    angLog1.append(q1*180/np.pi)
    angLog2.append(q2*180/np.pi)
    minE.append(False)
    

    if (np.abs(v0)+np.abs(v1)+np.abs(v2)) <tol and \
       (np.abs(error0)+np.abs(error1)+np.abs(error2)) <tol:
        targetORN = get_input()
        
            
        

    termTime = i+1
    p.stepSimulation()
    
# print("total time took (real time sim) : ", t1-t0)
    



''' PLOT ''' ''' PLOT ''' ''' PLOT ''' ''' PLOT ''' ''' PLOT ''' ''' PLOT ''' ''' PLOT '''



# # PLOT

# figure = plt.figure(figsize=[15, 4.5])
# figure.subplots_adjust(left=0.05, bottom=0.11, right=0.97, top=0.9, wspace=0.4, hspace=0.55)

# t = np.linspace(0,termTime/240,termTime)
# torqueLog0 = np.asarray(torqueLog0)
# torqueLog1 = np.asarray(torqueLog1)
# torqueLog2 = np.asarray(torqueLog2)
# errorLog0 = np.asarray(errorLog0)
# errorLog1 = np.asarray(errorLog1)
# errorLog2 = np.asarray(errorLog2)
# angLog0 = np.asarray(angLog0)
# angLog1 = np.asarray(angLog1)
# angLog2 = np.asarray(angLog2)
# minE = np.asarray(minE)


# ax_1 = figure.add_subplot(131)
# ax_1.set_title("TORQUE 0")
# ax_1.plot(t, torqueLog0, '-r')
# ax_1.grid()
# ax_1.set_xlabel("seconds")
# ax_1.set_ylabel("NM")

# ax_2 = figure.add_subplot(132)
# ax_2.set_title("TORQUE 1")
# ax_2.plot(t, torqueLog1, '-g')
# ax_2.grid()
# ax_2.set_xlabel("seconds")
# ax_2.set_ylabel("NM")

# ax_3 = figure.add_subplot(133)
# ax_3.set_title("TORQUE 2")
# ax_3.plot(t, torqueLog2, '-b')
# ax_3.grid()
# ax_3.set_xlabel("seconds")
# ax_3.set_ylabel("NM")

# plt.savefig("./results/torquePlot.png",bbox_inches='tight')


# figure = plt.figure(figsize=[15, 4.5])
# figure.subplots_adjust(left=0.05, bottom=0.11, right=0.97, top=0.9, wspace=0.4, hspace=0.55)

# mask = minE


# ax_4 = figure.add_subplot(131)
# ax_4.set_title("joint 0 error")
# ax_4.plot(t, errorLog0, '-r')
# ax_4.scatter(t[mask],errorLog0[mask],c='k',s=15)
# ax_4.grid()
# ax_4.set_xlabel("seconds")
# ax_4.set_ylabel("degrees")

# ax_5 = figure.add_subplot(132)
# ax_5.set_title("joint 1 error")
# ax_5.plot(t, errorLog1, '-g')
# ax_5.scatter(t[mask],errorLog1[mask],c='k',s=15)
# ax_5.grid()
# ax_5.set_xlabel("seconds")
# ax_5.set_ylabel("degrees")

# ax_6 = figure.add_subplot(133)
# ax_6.set_title("joint 2 error")
# ax_6.plot(t, errorLog2, '-b')
# ax_6.scatter(t[mask],errorLog2[mask],c='k',s=15)
# ax_6.grid()
# ax_6.set_xlabel("seconds")
# ax_6.set_ylabel("degrees")

# plt.savefig("./results/errorPlot.png",bbox_inches='tight')

# figure = plt.figure(figsize=[15, 4.5])
# figure.subplots_adjust(left=0.05, bottom=0.11, right=0.97, top=0.9, wspace=0.4, hspace=0.55)

# ax_7 = figure.add_subplot(131)
# ax_7.set_title("joint 0 angle")
# ax_7.plot(t, angLog0, '-r')
# ax_7.scatter(t[mask],angLog0[mask],c='k',s=20)
# ax_7.grid()
# ax_7.set_xlabel("seconds")
# ax_7.set_ylabel("degrees")

# ax_8 = figure.add_subplot(132)
# ax_8.set_title("joint 1 angle")
# ax_8.plot(t, angLog1, '-g')
# ax_8.scatter(t[mask],angLog1[mask],c='k',s=20)
# ax_8.grid()
# ax_8.set_xlabel("seconds")
# ax_8.set_ylabel("degrees")

# ax_8 = figure.add_subplot(133)
# ax_8.set_title("joint 2 angle")
# ax_8.plot(t, angLog2, '-b')
# ax_8.scatter(t[mask],angLog2[mask],c='k',s=20)
# ax_8.grid()
# ax_8.set_xlabel("seconds")
# ax_8.set_ylabel("degrees")

# plt.savefig("./results/anglePlot.png",bbox_inches='tight')

# plt.show()

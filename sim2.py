import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import argparse
import time

parser = argparse.ArgumentParser()
parser.add_argument('x', 
        type=float,
        help='1st target end effector world coordinate x')
parser.add_argument('y', 
        type=float,
        help='1st target end effector world coordinate y')
parser.add_argument('z', 
        type=float,
        help='1st target end effector world coordinate z')
args = parser.parse_args()

target = [args.x,args.y,args.z]

destination = [0.4,-0.4,args.z]

offsetJoint3 = -5*np.pi/180

tol = 1e-3

# Can alternatively pass in p.DIRECT 
client = p.connect(p.GUI)
p.setGravity(0, 0, -9.81, physicsClientId=client)


p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")



''' LOAD '''
flags = p.URDF_USE_SELF_COLLISION
# bodyId = p.loadURDF("./Arm_Final_Planet/urdf/Arm_Final_Planet.urdf", 
#                     basePosition=[0,0,0],useFixedBase=True,flags=flags)
# bodyId = p.loadURDF("./data/version2/Arm_Final_Planet/urdf/Arm_Final_Planet.urdf",
#                     basePosition=[0,0,0],useFixedBase=True,flags=flags)
bodyId = p.loadURDF("./data/Arm_Final_Planet_hook/urdf/Arm_Final_Planet_hook.urdf",
                    basePosition=[0,0,0],useFixedBase=True,flags=flags)
                    

''' LOAD '''

position, orientation = p.getBasePositionAndOrientation(bodyId)
numJoints = p.getNumJoints(bodyId)
print("number of joints = ",numJoints)
# for i in range(numJoints):
#     print("Joint ", i, "\n")
#     print(p.getJointInfo(bodyId,i))

mode = p.VELOCITY_CONTROL
maxForce = 10
targetVel = 4






torqueLog0 = []
torqueLog1 = []
torqueLog2 = []
errorLog0 = []
errorLog1 = []
errorLog2 = []
prev_error0 = 0
prev_error1 = 0
prev_error2 = 0
cumul_e0 = 0
cumul_e1 = 0
cumul_e2 = 0

kp0 = 2e-2
ki0 = 1e-8
kd0 = 4e02

kp1 = 3e-2
ki1 = 1e-7
kd1 = 5e02

kp2 = 2e-2
ki2 = 1e-4
kd2 = 4e02

print("0 -> kp,ki,ki = ",kp0,ki0,kd0)
print("1 -> kp,ki,ki = ",kp1,ki1,kd1)
print("2 -> kp,ki,ki = ",kp2,ki2,kd2)

prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
trailDuration = 15
hasPrevPose = 0

p.enableJointForceTorqueSensor(bodyId,0)
p.enableJointForceTorqueSensor(bodyId,1)
p.enableJointForceTorqueSensor(bodyId,2)
maxForce = 0

mode = p.VELOCITY_CONTROL 
p.setJointMotorControl2(bodyId, 0,controlMode=mode, force=maxForce)
p.setJointMotorControl2(bodyId, 1,controlMode=mode, force=maxForce)
p.setJointMotorControl2(bodyId, 2,controlMode=mode, force=maxForce)

ind,Name,tp,qInd,uInd,flags,damp, \
friction,ll,ul,maxF,maxV,linkName, \
ax,parPos,parOrn,ParInd = p.getJointInfo(bodyId,0)
# print("friction = ",friction)

p.changeDynamics(bodyId,0,jointLowerLimit = -np.pi,jointUpperLimit = np.pi)
p.changeDynamics(bodyId,1,jointLowerLimit = -np.pi,jointUpperLimit = np.pi)
p.changeDynamics(bodyId,2,jointLowerLimit = -np.pi,jointUpperLimit = np.pi)


ee_mass = p.getDynamicsInfo(bodyId,3)[0]
print("mass of end effector : ", ee_mass)
# print(p.getLinkState(bodyId,-1))


# print(p.getLinkState(bodyId,0))
# print(p.getLinkState(bodyId,1))
# print(p.getLinkState(bodyId,2))
# print("--------")
# print("--------")
# print("--------")
# print(p.getLinkState(bodyId,3))


targetORN = [np.pi/2,-np.pi/4,-np.pi/2]
# targetORN = [0,0,0]

# target = [0.4,0.2,0.1]

jointPos = p.calculateInverseKinematics(bodyId,3,target)

destPos = p.calculateInverseKinematics(bodyId,3,destination)

destination = destPos

if(target[0]<0):
    targetORN = [-np.pi +jointPos[0],-jointPos[1],-jointPos[2]]
else:
    targetORN = [np.pi +jointPos[0],-jointPos[1],-jointPos[2]]


print("target pos 0 (deg.):",targetORN[0]*180/np.pi)
print("target pos 1 (deg.):",targetORN[1]*180/np.pi)
print("target pos 2 (deg.):",targetORN[2]*180/np.pi)

state1 = False
a1 = False
state2 = False
a2 = False
state3 = False

payload = 0.1

duration = 50000


p.setRealTimeSimulation(0)
t0 = time.time()
for i in range(duration): 
    
    # pos, ori = p.getBasePositionAndOrientation(bodyId)

    if state1==False and a1 == False: 
        targetORN[2]+=offsetJoint3
        a1 = True
        print("Attempting at state 1 . . .")
    elif state1==True and state2==False and a2==False:
        targetORN[2]-=offsetJoint3
        a2 = True
        print("Attempting at state 2 . . .")
    elif state2==True:
        targetORN = destination


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
    # tau0,tau1,tau2 = p.calculateInverseDynamics(bodyId,
    #                                             [pos0,pos1,pos2],
    #                                             [vel0,vel1,vel2],
    #                                             [0,0,0])
    tau0,tau1,tau2 = p.calculateInverseDynamics(bodyId,
                                                [pos0,pos1,pos2],
                                                [0,0,0],
                                                [0,0,0])

    T0 = kp0*(error0) + kd0*(de0/240) + ki0*cumul_e0
    T1 = kp1*(error1) + kd1*(de1/240) + ki1*cumul_e1
    T2 = kp2*(error2) + kd2*(de2/240) + ki2*cumul_e2
    # print("torques 0 1 2: ",T0,",",T1,",",T2)

    prev_error0 = error0
    prev_error1 = error1
    prev_error2 = error2

    cumul_e0 += error0
    cumul_e1 += error1
    cumul_e2 += error2

    ####
    
    ####
    force0 = T0 + tau0
    force1 = T1 + tau1
    force2 = T2 + tau2
    
    p.setJointMotorControl2(bodyId,0,controlMode = p.TORQUE_CONTROL, force = force0)
    p.setJointMotorControl2(bodyId,1,controlMode = p.TORQUE_CONTROL, force = force1)
    p.setJointMotorControl2(bodyId,2,controlMode = p.TORQUE_CONTROL, force = force2)
    # p.setJointMotorControl2(bodyId,0,controlMode = p.POSITION_CONTROL, targetPosition = targetORN[0])
    # p.setJointMotorControl2(bodyId,1,controlMode = p.POSITION_CONTROL, targetPosition = targetORN[1])
    # p.setJointMotorControl2(bodyId,2,controlMode = p.POSITION_CONTROL, targetPosition = targetORN[2])
    # _,_,_,force0 = p.getJointState(bodyId,0)
    # _,_,_,force1 = p.getJointState(bodyId,0)
    # _,_,_,force2 = p.getJointState(bodyId,0)

    errorLog0.append(error0*180/np.pi)
    errorLog1.append(error1*180/np.pi)
    errorLog2.append(error2*180/np.pi)
    torqueLog0.append(force0)
    torqueLog1.append(force1)
    torqueLog2.append(force2)

    if (np.abs(v0)+np.abs(v1)+np.abs(v2)) <tol and \
       (np.abs(error0)+np.abs(error1)+np.abs(error2)) <tol:
        if state1==False: 
            state1 = True
            print("STATE CHANGED")
            print("reached state 1")
            cumul_e0 = 0
            cumul_e1 = 0
            cumul_e2 = 0
        elif state2==False: 
            state2 = True
            print("STATE CHANGED")
            print("reached state 2")
            cumul_e0 = 0
            cumul_e1 = 0
            cumul_e2 = 0
            p.changeDynamics(bodyId,3,mass = ee_mass+payload)
            # kp1*=3e03*payload
            # ki1*=3e03*payload
            # kd1*=3e03*payload
            # kp2*=1e02*payload
            # ki2*=1e02*payload
            # kd2*=1e02*payload
            kp1+=8e-01*payload
            ki1+=8e-01*payload
            kd1+=8e-01*payload
            kp2+=1e-03*payload
            ki2+=1e-03*payload
            kd2+=1e-03*payload
        else: 
            state3 = True
            print("reached state 3")
            t1 = time.time()
            
        


    if i%5000==0: 
        print(i)
        if(state2==True):
            print("Time elapsed: ",i/240," seconds")
            print("current errors (deg.): ",error0*180/np.pi,", ",
                                            error1*180/np.pi,", ",
                                            error2*180/np.pi)
    p.stepSimulation()
    
print("total time took (real time sim) : ", t1-t0)
    
# PLOT

figure = plt.figure(figsize=[15, 4.5])
figure.subplots_adjust(left=0.05, bottom=0.11, right=0.97, top=0.9, wspace=0.4, hspace=0.55)

t = np.linspace(0,duration/240,duration)
torqueLog0 = np.asarray(torqueLog0)
torqueLog1 = np.asarray(torqueLog1)
torqueLog2 = np.asarray(torqueLog2)
errorLog0 = np.asarray(errorLog0)
errorLog1 = np.asarray(errorLog1)
errorLog2 = np.asarray(errorLog2)

ax_1 = figure.add_subplot(131)
ax_1.set_title("TORQUE 0")
ax_1.plot(t, torqueLog0, '-r')
ax_1.grid()
ax_1.set_xlabel("seconds")
ax_1.set_ylabel("NM")

ax_2 = figure.add_subplot(132)
ax_2.set_title("TORQUE 1")
ax_2.plot(t, torqueLog1, '-g')
ax_2.grid()
ax_2.set_xlabel("seconds")
ax_2.set_ylabel("NM")

ax_3 = figure.add_subplot(133)
ax_3.set_title("TORQUE 2")
ax_3.plot(t, torqueLog2, '-b')
ax_3.grid()
ax_3.set_xlabel("seconds")
ax_3.set_ylabel("NM")


figure = plt.figure(figsize=[15, 4.5])
figure.subplots_adjust(left=0.05, bottom=0.11, right=0.97, top=0.9, wspace=0.4, hspace=0.55)

ax_4 = figure.add_subplot(131)
ax_4.set_title("joint 0 error")
ax_4.plot(t, errorLog0, '-r')
ax_4.grid()
ax_4.set_xlabel("seconds")
ax_4.set_ylabel("degrees")

ax_5 = figure.add_subplot(132)
ax_5.set_title("joint 1 error")
ax_5.plot(t, errorLog1, '-g')
ax_5.grid()
ax_5.set_xlabel("seconds")
ax_5.set_ylabel("degrees")

ax_6 = figure.add_subplot(133)
ax_6.set_title("joint 2 error")
ax_6.plot(t, errorLog2, '-b')
ax_6.grid()
ax_6.set_xlabel("seconds")
ax_6.set_ylabel("degrees")

plt.show()

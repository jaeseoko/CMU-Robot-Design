import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from Arm_Planet_IK import Arm_Planet_IK as customIK
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

parser.add_argument('--weight',
                    type = float,
                    help = 'set payload weight to pick up')
args = parser.parse_args()

target = [args.x,args.y,args.z]

destination = [-args.x,args.y,args.z]

offsetJoint3 = -10*np.pi/180

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



targetORN = np.asarray( p.calculateInverseKinematics(bodyId,3,target) )
                        # lowerLimits = [-np.pi, -np.pi/2,-np.pi/2],
                        # upperLimits = [np.pi, 0,0]  ) )

destORN = np.asarray( p.calculateInverseKinematics(bodyId,3,destination) )
                        # lowerLimits = [-np.pi, -np.pi/2,-np.pi/2],
                        # upperLimits = [np.pi, 0,0]  ) )

if(targetORN[1]>0):
    
    targetORN[1]*= -1
    print("inverting target joint 1 angle")
    targetORN[0]+=np.pi
    print("180 target rotating joint 0")
if(targetORN[2]>0):
    print("inverting target joint 2 angle")
    targetORN[2]*=-1

if(destORN[1]>0):
    
    destORN[1]*= -1
    print("inverting dest joint 1 angle")
    destORN[0]+=np.pi
    print("180 dest rotating joint 0")
if(destORN[2]>0):
    print("inverting dest joint 2 angle")
    destORN[2]*=-1




print("target pos 0 (deg.):",targetORN[0]*180/np.pi)
print("target pos 1 (deg.):",targetORN[1]*180/np.pi)
print("target pos 2 (deg.):",targetORN[2]*180/np.pi)

print("destination pos 0 (deg.):",destORN[0]*180/np.pi)
print("destination pos 1 (deg.):",destORN[1]*180/np.pi)
print("destination pos 2 (deg.):",destORN[2]*180/np.pi)

state1 = False
a1 = False
state2 = False
a2 = False
state3 = False
a3 = False
state4 = False
a4 = False
payload = args.weight

duration = 30000


p.setRealTimeSimulation(0)
t0 = time.time()
dt = 1/240
termTime = 0
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
    elif state2==True and state3==False and a3==False:
        targetORN = destORN
        a3 = True
        print("Attempting at state 3 . . .")
    elif state3==True and state4==False and a4==False:
        print("Reached state 3, attempting to place the load and disengage . . .")
        a4 = True
        targetORN[1]-=0.5*offsetJoint3
        targetORN[2]+=2*offsetJoint3


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

    tau0,tau1,tau2 = p.calculateInverseDynamics(bodyId,
                                                [q0,q1,q2],
                                                [v0,v1,v2],
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
    if error0 < 0.2 and error0 > -0.2:
        force0 = tau0
    elif error0 <0:
        force0 = tau0 - 0.1
    else:
        force0 = tau0 + 0.1

    if error1 < 0.2 and error1 > -0.2:
        force1 = tau1
    elif error1 <0:
        force1 = tau1 - 0.1
    else:
        force1 = tau1 + 0.1

    if error2 < 0.2 and error2 > -0.2:
        force2 = tau2
    elif error2 <0:
        force2 = tau2 - 0.1
    else:
        force2 = tau2 + 0.1

        
    ####
    # tau0 = 0
    # force0 = T0 + tau0
    # force1 = T1 + tau1
    # force2 = T2 + tau2
    
    p.setJointMotorControl2(bodyId,0,controlMode = p.TORQUE_CONTROL, force = force0)
    p.setJointMotorControl2(bodyId,1,controlMode = p.TORQUE_CONTROL, force = force1)
    p.setJointMotorControl2(bodyId,2,controlMode = p.TORQUE_CONTROL, force = force2)
    

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
        if state1==False: 
            state1 = True
            print("STATE CHANGED")
            print("reached state 1")
            minE[-1] = True
            
            # cumul_e0 = 0
            # cumul_e1 = 0
            # cumul_e2 = 0
        elif state2==False: 
            state2 = True
            print("STATE CHANGED")
            print("reached state 2")
            # cumul_e0 = 0
            # cumul_e1 = 0
            # cumul_e2 = 0
            p.changeDynamics(bodyId,3,mass = ee_mass+payload)
            minE[-1] = True

            kp0*=10.5*payload
            ki0*=10**5*payload
            kd0*=25*payload

            kp1*=1676.6*payload
            ki1*=10**6*payload
            kd1*=1010*payload

            kp2*=10.025*payload
            ki2*=10.5*payload
            kd2*=10.025*payload

            # kp0+=1e-02*payload
            # ki0+=1e-03*payload
            # kd0+=3e-01*payload

            # kp1+=500e-01*payload
            # ki1+=10e-02*payload
            # kd1+=400e-01*payload

            # kp2+=5e-04*payload
            # ki2+=5e-05*payload
            # kd2+=5e-04*payload
            print("gains:\n ")
            print(kp0,",",ki0,",",kd0)
            print(kp1,",",ki1,",",kd1)
            print(kp2,",",ki2,",",kd2)
            # 0.021 , 0.00010001 , 0.05
            # 5.03 , 0.010000100000000001 , 4.04
            # 0.020050000000000002 , 0.000105 , 0.020050000000000002
        elif state3== False: 
            state3 = True
            minE[-1] = True
            # t1 = time.time()
        elif state4 == False:
            state4 = True
            print("placed the load at final destination.")
            minE[-1] = True
            termTime=i+1
            break
            
        
        


    if i%5000==0: 
        print(i)
        if(state2==True):
            print("Time elapsed: ",i/240," seconds")
            print("current errors (deg.): ",error0*180/np.pi,", ",
                                            error1*180/np.pi,", ",
                                            error2*180/np.pi)
    termTime = i+1
    p.stepSimulation()
    
# print("total time took (real time sim) : ", t1-t0)
    



''' PLOT ''' ''' PLOT ''' ''' PLOT ''' ''' PLOT ''' ''' PLOT ''' ''' PLOT ''' ''' PLOT '''



# PLOT

figure = plt.figure(figsize=[15, 4.5])
figure.subplots_adjust(left=0.05, bottom=0.11, right=0.97, top=0.9, wspace=0.4, hspace=0.55)

t = np.linspace(0,termTime/240,termTime)
torqueLog0 = np.asarray(torqueLog0)
torqueLog1 = np.asarray(torqueLog1)
torqueLog2 = np.asarray(torqueLog2)
errorLog0 = np.asarray(errorLog0)
errorLog1 = np.asarray(errorLog1)
errorLog2 = np.asarray(errorLog2)
angLog0 = np.asarray(angLog0)
angLog1 = np.asarray(angLog1)
angLog2 = np.asarray(angLog2)
minE = np.asarray(minE)


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

plt.savefig("./results/torquePlot.png",bbox_inches='tight')


figure = plt.figure(figsize=[15, 4.5])
figure.subplots_adjust(left=0.05, bottom=0.11, right=0.97, top=0.9, wspace=0.4, hspace=0.55)

mask = minE


ax_4 = figure.add_subplot(131)
ax_4.set_title("joint 0 error")
ax_4.plot(t, errorLog0, '-r')
ax_4.scatter(t[mask],errorLog0[mask],c='k',s=15)
ax_4.grid()
ax_4.set_xlabel("seconds")
ax_4.set_ylabel("degrees")

ax_5 = figure.add_subplot(132)
ax_5.set_title("joint 1 error")
ax_5.plot(t, errorLog1, '-g')
ax_5.scatter(t[mask],errorLog1[mask],c='k',s=15)
ax_5.grid()
ax_5.set_xlabel("seconds")
ax_5.set_ylabel("degrees")

ax_6 = figure.add_subplot(133)
ax_6.set_title("joint 2 error")
ax_6.plot(t, errorLog2, '-b')
ax_6.scatter(t[mask],errorLog2[mask],c='k',s=15)
ax_6.grid()
ax_6.set_xlabel("seconds")
ax_6.set_ylabel("degrees")

plt.savefig("./results/errorPlot.png",bbox_inches='tight')

figure = plt.figure(figsize=[15, 4.5])
figure.subplots_adjust(left=0.05, bottom=0.11, right=0.97, top=0.9, wspace=0.4, hspace=0.55)

ax_7 = figure.add_subplot(131)
ax_7.set_title("joint 0 angle")
ax_7.plot(t, angLog0, '-r')
ax_7.scatter(t[mask],angLog0[mask],c='k',s=20)
ax_7.grid()
ax_7.set_xlabel("seconds")
ax_7.set_ylabel("degrees")

ax_8 = figure.add_subplot(132)
ax_8.set_title("joint 1 angle")
ax_8.plot(t, angLog1, '-g')
ax_8.scatter(t[mask],angLog1[mask],c='k',s=20)
ax_8.grid()
ax_8.set_xlabel("seconds")
ax_8.set_ylabel("degrees")

ax_8 = figure.add_subplot(133)
ax_8.set_title("joint 2 angle")
ax_8.plot(t, angLog2, '-b')
ax_8.scatter(t[mask],angLog2[mask],c='k',s=20)
ax_8.grid()
ax_8.set_xlabel("seconds")
ax_8.set_ylabel("degrees")

plt.savefig("./results/anglePlot.png",bbox_inches='tight')

plt.show()

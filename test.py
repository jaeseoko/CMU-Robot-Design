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
# client = p.connect(p.GUI)
client = p.connect(p.DIRECT)
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


q0 = np.pi/4
q1 = -np.pi/4
q2 = -np.pi/4
v0 = -1
v1 = 2
v2 = 2
print(p.calculateInverseDynamics(bodyId,
                                    [q0,q1,q2],
                                    [v0,v1,v2],
                                    [0,0,0]))
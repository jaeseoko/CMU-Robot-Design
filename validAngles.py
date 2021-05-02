import numpy as np
import pybullet as p
import argparse

client = p.connect(p.DIRECT)
p.setGravity(0, 0, -9.81, physicsClientId=client)
flags = p.URDF_USE_SELF_COLLISION
bodyId = p.loadURDF("./data/Arm_Final_Planet_hook/urdf/Arm_Final_Planet_hook.urdf",
                    basePosition=[0,0,0],useFixedBase=True,flags=flags)

parser = argparse.ArgumentParser()
parser.add_argument('x', 
                    type=float,
                    help='target end effector world coordinate x')
parser.add_argument('y', 
                    type=float,
                    help='target end effector world coordinate y')
parser.add_argument('z', 
                    type=float,
                    help='target end effector world coordinate z')


args = parser.parse_args()

target = [args.x,args.y,args.z]

targetORN = np.asarray( p.calculateInverseKinematics(bodyId,3,target) )
                        # lowerLimits = [-np.pi, -np.pi/2,-np.pi/2],
                        # upperLimits = [np.pi, 0,0]  ) )
if(targetORN[1]>0):
    targetORN[0]+=np.pi
    print("Rotating 180, joint 0")
    targetORN[1]*= -1
    print("inverting target joint 1 angle")
    
if(targetORN[2]>0):
    print("inverting target joint 2 angle")
    targetORN[2]*=-1

print("joint 0 target angle= ",targetORN[0]*180/np.pi)
print("joint 1 target angle= ",targetORN[1]*180/np.pi)
print("joint 2 target angle= ",targetORN[2]*180/np.pi)


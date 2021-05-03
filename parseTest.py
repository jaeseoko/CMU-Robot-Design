from time import sleep
import numpy as np

import pybullet as p
import argparse



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
    args = parser.parse_args()
    targetORN = [args.a0*np.pi/180,args.a1*np.pi/180,args.a2*np.pi/180]
    destORN = [args.a0*np.pi/180 + np.pi/2,args.a1*np.pi/180,args.a2*np.pi/180]
    prev_pos = [0,0,0]
    prev_error = [0,0,0]
    cum_e = [0,0,0]
    load = args.load
    picked, placed = False, False
    offset = False
    return targetORN,destORN,prev_pos,prev_error,cum_e,load,picked,placed,offset

def main():

    targetORN,destORN,prev_pos,prev_error,cum_e,load,picked,placed,offset = SetUp()
    for i in range(5):
        tau0,tau1,tau2 = p.calculateInverseDynamics(bodyId,[i*np.pi,i*(-np.pi/3),i*(-np.pi/4)],
                                            [2,2,0.5],
                                            [0,0,0])
        print("torques are: ")
        print(tau0,",",tau1,",",tau2)



main()
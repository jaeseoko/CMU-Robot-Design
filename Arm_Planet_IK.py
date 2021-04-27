import numpy as np
import copy
import argparse
#pose=[x,y,z,w,rx,ry,rz]
#x,y,z is the position of target pose(in m)
#w,rx,ry,rz is the orientation of target pose(quaternion)
#analytically calculates and returns the joint angles to reach target pose
#Note: currently ignores orientation due to limited DOF
#returns: [th1,th2,th3] in rad
#th1-3 are the joints
def Arm_Planet_IK(target_pose):
	pose=copy.deepcopy(target_pose)
	pose[2]=pose[2]-0.086
	#lengths of the links
	l_2=0.219
	l_3=0.240
	#basic check to see if pose is reachable
	dist=np.linalg.norm(pose[0:3])
	if dist>l_2+l_3:
		print("target pose not reachable by robot.")
		return None
	#calculate the rotation of the base joint
	th1=np.arctan2(pose[1],pose[0])-np.pi
	#draw a "triangle" using the arm links and distance
	#from joint 2 to ee, and use it to calculate remaining angles
	lift_th2=np.arctan2(pose[2],np.linalg.norm(pose[0:2]))
	small_th2=np.arccos((-l_3**2+dist**2+l_2**2)/(2*dist*l_2))
	th2=lift_th2+small_th2
	
	complement_th3=np.arccos((-dist**2+l_2**2+l_3**2)/(2*l_2*l_3))
	th3=np.pi-complement_th3

	return np.array([-th1,-th2,-th3])	
if __name__=="__main__":
	parser =argparse.ArgumentParser()
	parser.add_argument("x",type=float,help="enter x y z world coordinate")
	parser.add_argument("y",type=float,help="enter x y z world coordinate")
	parser.add_argument("z",type=float,help="enter x y z world coordinate")
	args = parser.parse_args()
	pose = [args.y,args.x,args.z]
	print(Arm_Planet_IK(pose) *180/np.pi, " in degrees." )

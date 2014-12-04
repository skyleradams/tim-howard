import numpy as np
import math
from pykalman import KalmanFilter
import sys,socket,time,struct,signal,serial,math

#### Prewritten functions for vision system ####
def signal_handler(signal, frame):
    global s1
    print('Closing Connection...')
    s1.close()
    sys.exit(0)

def getVals(sObj):
	sObj.send('s')
	raw = sObj.recv(16)
	x1 = struct.unpack('>H',raw[0:2])[0]
	y1 = struct.unpack('>H',raw[2:4])[0]
	a1 = struct.unpack('>H',raw[4:6])[0]
	x2 = struct.unpack('>H',raw[6:8])[0]
	y2 = struct.unpack('>H',raw[8:10])[0]
	a2 = struct.unpack('>H',raw[10:12])[0]
	timestamp = struct.unpack('>f',raw[12:16])[0]
	return (x1, y1, a1, x2, y2, a2, timestamp)


#### Own functions ####

def initKalman(initstate, initcovariance):
	deltaT_default = 1.0/60 #60Hz frames

	Transition_Matrix, b = transitionMatrices(deltaT_default,initstate)
	Observation_Matrix=[[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0]]

	posCov = 5e-4
	velCov = 1e-4
	transistionCov=np.diag([posCov,posCov,posCov,velCov,velCov,velCov]) #how confident are we of our model..how about impact cases. more certain about pos than speed?
	observationCov=0.5e-2*np.eye(3) #measure from data, may change according to blob size
	kf=KalmanFilter(transition_matrices=Transition_Matrix,
            observation_matrices=Observation_Matrix,
            initial_state_mean=initstate,
            initial_state_covariance=initcovariance,
            transition_covariance=transistionCov,
            observation_covariance=observationCov)

	return kf

def transitionMatrices(deltaT,state):
	g = 9.81
	pz = state[2]
	vz = state[5]
	pz_tol = 0.05 #height at which rolling is assumed
	vz_tol = math.sqrt(2*g*pz_tol)*0.5
	Transition_Matrix = np.zeros((6,6))
	
	if ((pz < pz_tol) and (math.fabs(vz) < vz_tol)):
		#rolling case
		dragxy = 0.1
		zconvergence = 0.1
		Transition_Matrix[:,:]=[[1,0,0,deltaT,0,0],[0,1,0,0,deltaT,0],[0,0,1-zconvergence,0,0,deltaT],[0,0,0,(1-dragxy*deltaT),0,0],[0,0,0,0,(1-dragxy*deltaT),0],[0,0,0,0,0,0]]
		inputVector = None
	elif ((pz + deltaT*vz - g*0.5*deltaT**2 < 0) and (vz < 0)):
		#bouncing - impact case
		epsilon = 0.7
		Transition_Matrix[:,:]=[[1,0,0,deltaT,0,0],[0,1,0,0,deltaT,0],[0,0,0,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,-epsilon]]
		inputVector = None
	else:
		#bouncing - inflight case
		Transition_Matrix[:,:]=[[1,0,0,deltaT,0,0],[0,1,0,0,deltaT,0],[0,0,1,0,0,deltaT],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]]
		inputVector = np.zeros(6)
		inputVector[2] = -g*0.5*(deltaT**2)
		inputVector[5] = -g*deltaT

	return Transition_Matrix, inputVector

def cameraTransform( x1, y1, a1, x2, y2, a2 ):
	# Reads input from two cameras in pixels, provides ball position in world 
	# coordinate system in SI coordinates; assumes no distortion; assumes
	# cameras are centered; assume we are given x_pic1, y_pic1, crop_x_top, 
	# xrop_x_bottom crop_y_left, crop_y_right; assume we can calculate f_2 from
	# analyzing reference images; assume we are given position of camera 2 in
	# world coorinates, x_cam2, y_cam2, z_cam2; assume we are given x_pic2 and
	# that "flat" (constant world z) is in the middle of the camera


	#From the other code's POV, the incoming coordinates are 0,0,0 from the opposite corner of what
	# our code assumes. I.E. Facing the goal, our origin is the left corner vs the code assumes the right corner is the origin.
	#crop_x_top = 0
	#crop_x_bottom = 0
	#crop_y_left = 0
	#crop_y_right = 0
	inToM = 0.0254
	#The overall pixel lengths (size of picture)
	x_pic1 = 664
	x_pic2 = 752
	y_pic1 = 472
	y_pic2 = 480

	#Adjust
	#y1_adj = 472 - y1 #pixels
	y1_adj = y1 #silly me
	z = 480 - y2
    

	x_cam2 = 0
	y_cam2 = 0
	z_cam2 = 0
	f_2 = 1

	#r_x1 = (10 / 3.28084) / (x_pic1 - crop_x_top - crop_x_bottom);
	#r_y1 = (8 / 3.28084) / (y_pic1 - crop_y_left - crop_y_right);
	#x_ball = (x1 - crop_x_top) * r_x1;
	#y_ball = (y1 - crop_y_left) * r_y1;

	#According to a linear interpolation of the calibration data
	#x_ball = (0.1981*x1 + 6.64643)*inToM #[m] depends only on pixels in x direction
	#x_ball = (6.523 + 0.1981*x1 + 0.0004668*y1_adj)*inToM  #depends on pixels in x and y
	x_ball = (8.66 + 0.1905*x1 - 0.01139*y1_adj + 7.66e-6*x1**2 + 1.086e-5*x1*y1_adj + 1.741e-5*y1_adj**2)*inToM #more accurate calc depending on both x and y dire
    
	#follows same pattern as x ball
	#y_ball = (0.195*y1_adj + 1.3193)*2.54/100 #[m]
	#y_ball = (2.258 - 0.002925*x1 + 0.1951*y1_adj)*inToM
	y_ball = (2.701 - 0.005226*x1 + 0.1928*y1_adj + 3.707e-6*x1**2 - 5.647e-7*x1*y1_adj + 5.216e-6*y1_adj**2)*inToM

	#d_xy = math.sqrt((x_ball-x_cam2)**2+(y_ball-y_cam2)**2);
	#z_ball = z_cam2 - (x2-(x_pic2)/2) / f_2 * d_xy;
	#using fit() from matlab, zball = f(z_pixels, xball, yball), but we need to make it a function of only two variables. 
	#I propose that alpha = atan2(yball, xball), and that will be a property of the zball coordinate.
	alpha = math.atan2(y_ball, x_ball)
	#z_ball = -19.98 + 3.423*alpha + .09715*z
	#z_ball = -68.96 - 26.4*alpha + .4684*z + 2.762*alpha*alpha + .0691*alpha*z - .0006249*z*z
	z_ball = 0
	return [x_ball, y_ball, z_ball]

def packInteger(value):
    """Packs a python 2 byte int integer to an arduino int long"""
    return struct.pack('h', value)    #should check bounds

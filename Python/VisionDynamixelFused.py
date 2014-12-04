import os, platform
import dynamixel
import time
import options
import math
import serial
import numpy as np
import goalieFunctions
from matplotlib import pyplot as plt
from matplotlib.pylab import subplots,close
from mpl_toolkits.mplot3d import Axes3D
import sys,socket,struct,signal

''' Definitions and initialization DynamixelControl '''

ticks_per_rad = 4096.0/(math.pi*2)

############################################
#  _______         __ ______                     __
# /_  __(_)_ _    / // / __ \_    _____ ________/ /
#  / / / /  ' \  / _  / /_/ / |/|/ / _ `/ __/ _  / 
# /_/ /_/_/_/_/ /_//_/\____/|__,__/\_,_/_/  \_,_/  
############################################

myActuators = list()


def forwardKinematics(theta1, theta2, l1, l2):
    return [l1*math.cos(theta1)+l2*(math.cos(theta1+theta2)),
            l1*math.sin(theta1)+l2*(math.sin(theta1+theta2))]
#Given: xE,yE, l1, l2
#Return: theta1,theta2
def inverseKinematics(xIn, yIn, l1, l2):
	myTheta2 = 2*math.atan2(math.sqrt(((l1+l2)**2-(xIn**2+yIn**2))),math.sqrt((xIn**2+yIn**2.0)-(l1-l2)**2))
	myTheta1 = math.atan2(yIn,xIn)-math.atan2(l2*math.sin(myTheta2),l1+l2*math.cos(myTheta2))
	return (scaleToCircle(myTheta1), scaleToCircle(myTheta2))

def computeAltIK(x, y, theta1, theta2):
	#theta1 and 2 are IK outputs
	t2 = -theta2
	angle_to_endpoint = scaleToCircle(math.atan2(y,x))
	if angle_to_endpoint > theta1:
		t1 = theta1 + 2*(angle_to_endpoint-theta1)
	elif angle_to_endpoint < theta1:
		t1 = theta1 + 2*(angle_to_endpoint-theta1)
	else:
		t1 = theta1

	return (t1, t2)

def scaleToCircle(radianvalue):
	return radianvalue % (2*math.pi)


def boundWithinGoal(value, upper, lower):
	if value > upper:
		bounded = upper
	elif value < lower:
		bounded = lower
	else:
		bounded = value
	return bounded

def boundWithinRobotReach(x, y, radius):
	if math.sqrt(math.pow(x,2)+math.pow(y,2)) > radius:
		angle = math.atan2(y,x)
		return (0.98*radius*math.cos(angle), 0.98*radius*math.sin(angle))
	else:
		return (x,y)
def withinThreshold(difference, thresh):
	if abs(difference) <= thresh:
		return True
	elif abs(abs(difference)-2*math.pi) <= thresh:
		return True
	else:
		return False

def actuatorsMoving(actuators):
    for actuator in actuators:
        if actuator.cache[dynamixel.defs.REGISTER['Moving']]:
            return True
    return False

if platform.dist()[0] == 'Ubuntu':
    portName = options.ubuntu_port
elif os.name == "posix":
    portName = options.unix_port
else:
    portName = options.windows_port

serial = dynamixel.serial_stream.SerialStream( port=portName, baudrate=options.baudrate, timeout=1)
net = dynamixel.dynamixel_network.DynamixelNetwork( serial )
net.scan( 1, options.num_servos )

print "Scanning for Dynamixels...",
for dyn in net.get_dynamixels():
    print dyn.id,
    myActuators.append(net[dyn.id])

print "FOUND:" + str(myActuators)

for actuator in myActuators:
	actuator.moving_speed = options.servo_speed
	actuator.synchronized = True
	actuator.torque_enable = True
	actuator.torque_control_enable = False
	actuator.torque_limit = 1024
	actuator.max_torque = 1024


class Arm(object):

	def __init__(self, shoulder, elbow, params):
		self.params = params
		self.shoulder = shoulder
		self.elbow = elbow
		self.elbow_angle = 0
		self.shoulder_angle = 0
		#motors

	def update(self):
		net.synchronize()
		self.shoulder.read_all()
		self.elbow.read_all()

	def moveToXY(self,x,y):
		theta1, theta2 = inverseKinematics(x,y, self.params.l1, self.params.l2)
		(shoulderCurr, elbowCurr) = self.returnCurrentPositions()
		(shoulderCurrNOMOD, elbowCurrNOMOD) = self.returnCurrentPositionsNOMOD()

		alpha = shoulderCurr - theta1
		if abs(alpha) > abs(shoulderCurr - (theta1+2*math.pi)):
			alpha = shoulderCurr - (theta1+2*math.pi)
		if abs(alpha) > abs(shoulderCurr - (theta1-2*math.pi)):
			alpha = shoulderCurr - (theta1-2*math.pi)

		beta = elbowCurr - theta2
		if abs(beta) > abs(elbowCurr - (theta2+2*math.pi)):
			beta = elbowCurr - (theta2+2*math.pi)
		if abs(beta) > abs(elbowCurr - (theta2-2*math.pi)):
			beta = elbowCurr - (theta2-2*math.pi)

		self.moveToTheta(shoulderCurrNOMOD-alpha, elbowCurrNOMOD-beta)


	def moveToXYGoal(self, x, y):
		x, y = Arm.transformGoaltoRobot(self,x,y)
		x, y = boundWithinRobotReach(x,y, self.params.l1+self.params.l2)
		x = boundWithinGoal(x, self.params.max_x, self.params.min_x)
		y = boundWithinGoal(y, self.params.max_y, self.params.min_y)
		self.moveToXY(x,y)


	def transformGoaltoRobot(self,x,y):
		return (x-self.params.horizontal_offset, y-self.params.vertical_offset)


	def moveToTheta(self, t1, t2):
		#print t1, t2
		self.shoulder_angle = t1
		self.elbow_angle = t2
		self.shoulder.goal_position = int((self.shoulder_angle*ticks_per_rad)+self.params.shoulder_offset)
		self.elbow.goal_position = int(((self.elbow_angle*ticks_per_rad) +self.params.elbow_offset)/2)


	def isMoving(self):
	    for actuator in [self.shoulder, self.elbow]:
	        if actuator.cache[dynamixel.defs.REGISTER['Moving']]:
	            return True
	    return False


	def returnCurrentPositions(self):
		theta1 = (self.shoulder.cache[dynamixel.defs.REGISTER['CurrentPosition']]-self.params.shoulder_offset)/ticks_per_rad
		theta2 = (self.elbow.cache[dynamixel.defs.REGISTER['CurrentPosition']]-self.params.elbow_offset)/ticks_per_rad*2
		theta1 = scaleToCircle(theta1)
		theta2 = scaleToCircle(theta2)
		return (theta1, theta2)

	def returnCurrentPositionsNOMOD(self):
		theta1 = (self.shoulder.cache[dynamixel.defs.REGISTER['CurrentPosition']]-self.params.shoulder_offset)/ticks_per_rad
		theta2 = (self.elbow.cache[dynamixel.defs.REGISTER['CurrentPosition']]-self.params.elbow_offset)/ticks_per_rad*2
		return (theta1, theta2)

	def nearGoalPosition(self):
		shoulder, elbow = Arm.returnCurrentPositions(self)
		if withinThreshold(scaleToCircle(shoulder-self.shoulder_angle),self.params.angle_threshold) and withinThreshold(scaleToCircle(elbow-self.elbow_angle),self.params.angle_threshold):
			return True
		else:
			return False
		



a_arm = Arm(myActuators[0], myActuators[1], options.left_arm)
b_arm = Arm(myActuators[2], myActuators[3], options.right_arm)
a_arm.update()
b_arm.update()

'''Definitions and initialization vision'''

########  Set up TCP/IP Connection #### (prewritten code for vision system)
serverIP = '192.168.1.212'  #Use with the 2.12 Servers
#serverIP = 'localhost'      #Use for loopback testing on your own computer
s1=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
try:
    s1.connect((serverIP,2121))
    print 'Connecting to vision server...'
except socket.error:
    print('connection error vision server')
    sys.exit(0)

#### variable definitions and initialization ####

pxGoalie = 0.46 #[m] Position of goalie
deltaT_predictor = 0.1 #timestep for prediction... accuracy vs speed
predictTimelimit = 4; #[s] Time limit to abort kalman predictor

#tolerances for sending values to OpenCM
tTol = 0.2
distTol = 0.05
t_goal_prev = 0
y_goal_prev = 0
z_goal_prev = 0

timevec = list()#[] #initialize time array
pxRaw, pxFilt, pyRaw, pyFilt, pzRaw, pzFilt, vxRaw, vxFilt, vyRaw, vyFilt, vzRaw, vzFilt = [], [], [], [], [], [], [], [], [], [], [], [] #initialize arrays to store data

#initialize the kalman filter
xinit=0
yinit=0
zinit=0
vxinit=0
vyinit=0
vzinit=0
initstate = [xinit,yinit,zinit,vxinit,vyinit,vzinit]
initcovariance = 5.0e-1*np.eye(6) #initial speed covariance very large, position better known?!

kf = goalieFunctions.initKalman(initstate, initcovariance)
prev_filtered_state_mean = initstate
prev_filtered_state_covariance = initcovariance

RawCameraData = []


#start plot commands
# realtime_fig,realtime_ax = subplots(1,1)
# realtime_ax.set_aspect('equal')
# realtime_ax.set_xlim(-2,4)
# realtime_ax.set_ylim(-2,4)
# realtime_ax.set_xlabel("x [m]")
# realtime_ax.set_ylabel("y [m]")
# realtime_ax.set_title("Live plot of x-y plane")
# realtime_ax.hold(True)
# realtime_fig.canvas.draw()
# background = realtime_fig.canvas.copy_from_bbox(realtime_ax.bbox)
# realtime_plothandle_measurement = realtime_ax.plot(xinit,yinit,'xr', label='measured')[0]
# realtime_plothandle_kalman = realtime_ax.plot(xinit,yinit,'ob', label='kalman output')[0]
# plt.legend(loc=2)
# plt.ion()
# plt.show()
#end plot commands


raw_input("Press any key to start")

''' Runtime Loop '''
tic = time.time()
loopcount = 0
loopRuntime = 0
(x1, y1, a1, x2, y2, a2, timestamp_prev) = goalieFunctions.getVals(s1) #initialzise previous timestamp

while True:
	try:
		(x1, y1, a1, x2, y2, a2, timestamp) = goalieFunctions.getVals(s1) #grab frame from camera
		if a1 == 0:
			continue
		#(x1, y1, a1, x2, y2, a2, timestamp) = SavedData[loopcount]
		camera_observation = goalieFunctions.cameraTransform( x1, y1, a1, x2, y2, a2) #transfrom to our system
		if camera_observation[0] > 2.8 or camera_observation[0] < 0.39: #throw out measurements beyond striker point
			continue

		timestamp_current = timestamp
		deltaT = timestamp_current-timestamp_prev
		timestamp_prev = timestamp_current #update

	
		if deltaT == 0: #camera gave us old frame that we already have -- discard and try again
			continue

		A, b = goalieFunctions.transitionMatrices(deltaT,prev_filtered_state_mean)
		[next_filtered_state_mean, next_filtered_state_covariance] = kf.filter_update(prev_filtered_state_mean, prev_filtered_state_covariance, observation=camera_observation, transition_matrix=A, transition_offset=b, transition_covariance=None, observation_matrix=None, observation_offset=None, observation_covariance=None)

	
		# predicting target position on goal (may implement to execute less frequently than whole loop)
		px = next_filtered_state_mean[0]
		vx = next_filtered_state_mean[3]
		if vx < 0 and px > pxGoalie+0.3: #if not, ball not heading towards goal or is behind it
			predictionStep = 0
			predictedState = next_filtered_state_mean
			predictedCovariance = next_filtered_state_covariance
			while predictedState[0] >= pxGoalie and predictionStep*deltaT_predictor < predictTimelimit:
				#do kalman predition steps without measurement
				A,b = goalieFunctions.transitionMatrices(deltaT_predictor,predictedState)
				[predictedState, predictedCovariance] = kf.filter_update(predictedState, predictedCovariance, observation=None, transition_matrix=A, transition_offset=b, transition_covariance=None, observation_matrix=None, observation_offset=None, observation_covariance=None)
				predictionStep += 1

			t_goal = deltaT_predictor*predictionStep #time from point of prediction start to time of goal
			y_goal = predictedState[1]
			z_goal = predictedState[2]
		
			#Test if new prediction is sufficiently away from previous. If yes, send to Dynamixel
			if abs(t_goal-t_goal_prev)>tTol or abs(y_goal-y_goal_prev)>distTol or abs(z_goal-z_goal_prev)>distTol:
				y_goalcoords_in = (y_goal*39.37)-18.0
				z_goalcoords_in = (z_goal*39.37)

				goal = [y_goalcoords_in, z_goalcoords_in]
				print "moving to"+str(goal)
				(theta1_left, theta2_left) = a_arm.returnCurrentPositions()
				(theta1_right, theta2_right) = b_arm.returnCurrentPositions()

				currXY_left = forwardKinematics(theta1_left, theta2_left, options.left_arm.l1, options.left_arm.l2) #in robot coords
				currXY_left_world = [currXY_left[0]+options.left_arm.horizontal_offset, currXY_left[1]+options.left_arm.vertical_offset]
				gamma_left = math.atan2(goal[1]-currXY_left_world[1], goal[0]-currXY_left_world[0])
				currXY_right = forwardKinematics(theta1_right, theta2_right, options.right_arm.l1, options.right_arm.l2) #in robot coords
				currXY_right_world = [currXY_right[0]+options.right_arm.horizontal_offset, currXY_right[1]+options.right_arm.vertical_offset]
				gamma_right = math.atan2(goal[1]-currXY_right_world[1], goal[0]-currXY_right_world[0])

				l_left=4
				l_right=4
				if( ((goal[1]-currXY_left_world[1])**2 + (goal[0]-currXY_left_world[0])**2) < l_left**2):
					l_left = math.sqrt((goal[1]-currXY_left_world[1])**2 + (goal[0]-currXY_left_world[0])**2)
				if ( ((goal[1]-currXY_right_world[1])**2 + (goal[0]-currXY_right_world[0])**2) < l_right**2):
					l_right = math.sqrt((goal[1]-currXY_right_world[1])**2 + (goal[0]-currXY_right_world[0])**2)

				a_arm.moveToXYGoal(currXY_left_world[0]+l_left*math.cos(gamma_left), currXY_left_world[1]+l_left*math.sin(gamma_left))
				b_arm.moveToXYGoal(currXY_right_world[0]+l_right*math.cos(gamma_right), currXY_right_world[1]+l_right*math.sin(gamma_right))

				a_arm.update()
				b_arm.update()
				


		#start live plotting:
		# realtime_plothandle_kalman.set_data(next_filtered_state_mean[0],next_filtered_state_mean[1])
		# realtime_plothandle_measurement.set_data(camera_observation[0],camera_observation[1])
		# realtime_fig.canvas.restore_region(background)
		# realtime_ax.draw_artist(realtime_plothandle_kalman)
		# realtime_ax.draw_artist(realtime_plothandle_measurement)
		# realtime_fig.canvas.blit(realtime_ax.bbox)
		#end live plotting

		#update for next loop
		prev_filtered_state_mean = next_filtered_state_mean
		prev_filtered_state_covariance = next_filtered_state_covariance
	
		loopcount += 1
		loopRuntime += deltaT

	except KeyboardInterrupt:
		break

toc = time.time() - tic
print("Time per loop iteration is " + str(toc/loopcount) + " s. Corresponds to " + str(loopcount/toc) + " Hz.")
s1.close() #disconnect from vision network 

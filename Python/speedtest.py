import os, platform
import dynamixel
import time
import options
import math
import serial

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
net.scan( 1, 2 )

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
		self.elbow.goal_position = self.elbow.current_position
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
		



a = Arm(myActuators[0], myActuators[1], options.left_arm)
a.update()


for actuator in myActuators:
	actuator.moving_speed = 50

leftShoulder = myActuators[0]
leftElbow = myActuators[1]



a.moveToTheta(math.pi/2, math.pi)
a.update()
time.sleep(2)



for actuator in myActuators:
	actuator.moving_speed = options.servo_speed

for x in range(10):
	a.moveToTheta(.98, math.pi+.98)
	a.update()
	time.sleep(.2)
	a.moveToTheta(2, math.pi-.98)
	a.update()
	time.sleep(.2)


# raw_input("Press any key to start")
# print "Press ctrl+c to start rotation"
# while True:
# 	try:

# 		(theta1_left, theta2_left) = a.returnCurrentPositions()
# 		#(theta1_right, theta2_right) = b.returnCurrentPositions()

# 		currXY_left = forwardKinematics(theta1_left, theta2_left, options.left_arm.l1, options.left_arm.l2) #in robot coords
# 		currXY_left_world = [currXY_left[0]+options.left_arm.horizontal_offset, currXY_left[1]+options.left_arm.vertical_offset]
# 		gamma_left = math.atan2(goal[1]-currXY_left_world[1], goal[0]-currXY_left_world[0])
# 		#currXY_right = forwardKinematics(theta1_right, theta2_right, options.right_arm.l1, options.right_arm.l2) #in robot coords
# 		#currXY_right_world = [currXY_right[0]+options.right_arm.horizontal_offset, currXY_right[1]+options.right_arm.vertical_offset]
# 		#gamma_right = math.atan2(goal[1]-currXY_right_world[1], goal[0]-currXY_right_world[0])

# 		l_left=4
# 		l_right=4
# 		if( ((goal[1]-currXY_left_world[1])**2 + (goal[0]-currXY_left_world[0])**2) < l_left**2):
# 			l_left = math.sqrt((goal[1]-currXY_left_world[1])**2 + (goal[0]-currXY_left_world[0])**2)
# 		#if ( ((goal[1]-currXY_right_world[1])**2 + (goal[0]-currXY_right_world[0])**2) < l_right**2):
# 		#	l_right = math.sqrt((goal[1]-currXY_right_world[1])**2 + (goal[0]-currXY_right_world[0])**2)

# 		a.moveToXYGoal(currXY_left_world[0]+l_left*math.cos(gamma_left), currXY_left_world[1]+l_left*math.sin(gamma_left))
# 		#b.moveToXYGoal(currXY_right_world[0]+l_right*math.cos(gamma_right), currXY_right_world[1]+l_right*math.sin(gamma_right))
# 		a.update()
# 		#b.update()
		
		
# 	except KeyboardInterrupt:
# 		print "Starting rotation..."
# 		break

# #activate wheel mode
# leftElbow.ccw_angle_limit = 0
# leftElbow.cw_angle_limit = 0
# leftElbow.moving_speed = 20
# net.synchronize()
# #a.update()

# time.sleep(2)
# leftElbow.moving_speed = 1
# #a.update()
# net.synchronize()

# #activate multiturn mode again
# leftElbow.ccw_angle_limit = 4095
# leftElbow.cw_angle_limit = 4095
# net.synchronize()


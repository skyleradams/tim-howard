import os
import dynamixel
import time
import options
import math
import serial

############################################
#  _______         __ ______                     __
# /_  __(_)_ _    / // / __ \_    _____ ________/ /
#  / / / /  ' \  / _  / /_/ / |/|/ / _ `/ __/ _  / 
# /_/ /_/_/_/_/ /_//_/\____/|__,__/\_,_/_/  \_,_/  
############################################

myActuators = list()


def forwardKinematics(theta1, theta2, l1, l2):
    return [l1*math.cos(theta1)+l2*(math.cos(theta1)+math.cos(theta2)),
            l1*math.sin(theta1)+l2*(math.sin(theta1)+math.sin(theta2))]
#Given: xE,yE, l1, l2
#Return: theta1,theta2
def inverseKinematics(xIn, yIn, l1, l2):
    myTheta2 = 2*math.atan2(math.sqrt(((l1+l2)**2-(xIn**2+yIn**2))),math.sqrt((xIn**2+yIn**2.0)-(l1-l2)**2))
    myTheta1 = math.atan2(yIn,xIn)-math.atan2(l2*math.sin(myTheta2),l1+l2*math.cos(myTheta2))
    return (myTheta1, myTheta2)

def initializeDynamixels():
	if os.name == "posix":
	    portName = options.unix_port
	else:
	    portName = options.windows_port


	print "Scanning for Dynamixels...",
	for dyn in net.get_dynamixels():
	    print dyn.id,
	    myActuators.append(net[dyn.id])
	print "...Done"




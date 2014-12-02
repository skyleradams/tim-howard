#main program goalie robot

import numpy as np
import goalieFunctions
from matplotlib import pyplot as plt
from matplotlib.pylab import subplots,close
from mpl_toolkits.mplot3d import Axes3D
import time
import sys,socket,struct,signal,serial

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

#### Setup USB Connection to OpenCM ####
'''
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=9600,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS,
    timeout=None
)
ser.isOpen()
'''

#### variable definitions and initialization ####

pxGoalie = 0.2 #[m] Position of goalie
deltaT_predictor = 0.1 #timestep for prediction... accuracy vs speed
predictTimelimit = 4; #[s] Time limit to abort kalman predictor
goalwidth = 0.7 #[m]
goalheight = 1 #[m]

#tolerances for sending values to OpenCM
tTol = 0.2
distTol = 0.05
t_goal_prev = 0
y_goal_prev = 0
z_goal_prev = 0

timevec = [] #initialize time array
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

numframes = 500
RawCameraData = []


#start plot commands
realtime_fig,realtime_ax = subplots(1,1)
realtime_ax.set_aspect('equal')
realtime_ax.set_xlim(-3,3)
realtime_ax.set_ylim(-3,3)
realtime_ax.set_xlabel("x [m]")
realtime_ax.set_ylabel("y [m]")
realtime_ax.set_title("Live plot of x-y plane")
realtime_ax.hold(True)
realtime_fig.canvas.draw()
background = realtime_fig.canvas.copy_from_bbox(realtime_ax.bbox)
realtime_plothandle_measurement = realtime_ax.plot(xinit,yinit,'xr', label='measured')[0]
realtime_plothandle_kalman = realtime_ax.plot(xinit,yinit,'ob', label='kalman output')[0]
plt.legend(loc=2)
plt.ion()
plt.show()
#end plot commands


#### Runtime loop ####
tic = time.time()
loopcount = 0
loopRuntime = 0
(x1, y1, a1, x2, y2, a2, timestamp_prev) = goalieFunctions.getVals(s1) #initialzise previous timestamp
#SavedData=np.load("RawCameraData_RestingBall_1000.npy")
#(x1, y1, a1, x2, y2, a2, timestamp_prev) = SavedData[0]
while True:
	
	if loopcount >= numframes:
		break

	(x1, y1, a1, x2, y2, a2, timestamp) = goalieFunctions.getVals(s1) #grab frame from camera
	#(x1, y1, a1, x2, y2, a2, timestamp) = SavedData[loopcount]

	timestamp_current = timestamp
	deltaT = timestamp_current-timestamp_prev
	timestamp_prev = timestamp_current #update

	
	if deltaT == 0: #camera gave us old frame that we already have -- discard and try again
		continue

	RawCameraData.append((x1, y1, a1, x2, y2, a2, timestamp))
	camera_observation = goalieFunctions.cameraTransform( x1, y1, a1, x2, y2, a2) #transfrom to our system
	
	pxRaw.append(camera_observation[0])
	pyRaw.append(camera_observation[1])
	pzRaw.append(camera_observation[2])
	
	A, b = goalieFunctions.transitionMatrices(deltaT,prev_filtered_state_mean)
	[next_filtered_state_mean, next_filtered_state_covariance] = kf.filter_update(prev_filtered_state_mean, prev_filtered_state_covariance, observation=camera_observation, transition_matrix=A, transition_offset=b, transition_covariance=None, observation_matrix=None, observation_offset=None, observation_covariance=None)

	pxFilt.append(next_filtered_state_mean[0])
	pyFilt.append(next_filtered_state_mean[1])
	pzFilt.append(next_filtered_state_mean[2])

	
	# predicting target position on goal (may implement to execute less frequently than whole loop)
	px = next_filtered_state_mean[0]
	vx = next_filtered_state_mean[3]
	if vx < 0 and px > pxGoalie: #if not, ball not heading towards goal or is behind it
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

		'''
		#Test if new prediction is sufficiently away from previous. If yes, send to openCM
		if abs(t_goal-t_goal_prev)>tTol or abs(y_goal-y_goal_prev)>distTol or abs(z_goal-z_goal_prev)>distTol:
			#now send goal position to robot if OpenCM is ready
			if ser.inWaiting() > 0 or loopcount == 0:
				out = ''
				if loopcount != 0:
					out = ser.read(1)
				else:
					out = str(unichr(6))
			
				if out == str(unichr(6)):
					ser.write(goalieFunctions.packInteger(t_goal*100)+goalieFunctions.packInteger(y_goal*100)+goalieFunctions.packInteger(z_goal*100))
					print("Sent new goal position")
					#sent was successfull, hence update prev values
					t_goal_prev = t_goal
					y_goal_prev = y_goal
					z_goal_prev = z_goal
				else:
					print("Was ready but received " + out)
			else:
				print("OpenCM was not ready")
		'''


	#start live plotting:
	realtime_plothandle_kalman.set_data(next_filtered_state_mean[0],next_filtered_state_mean[1])
	realtime_plothandle_measurement.set_data(camera_observation[0],camera_observation[1])
	realtime_fig.canvas.restore_region(background)
	realtime_ax.draw_artist(realtime_plothandle_kalman)
	realtime_ax.draw_artist(realtime_plothandle_measurement)
	realtime_fig.canvas.blit(realtime_ax.bbox)
	#end live plotting

	#update for next loop
	prev_filtered_state_mean = next_filtered_state_mean
	prev_filtered_state_covariance = next_filtered_state_covariance
	
	loopcount += 1
	timevec.append(loopRuntime)
	loopRuntime += deltaT

	#TODO: implement break mechanism to loop
toc = time.time() - tic
print("Time per loop iteration is " + str(toc/loopcount) + " s. Corresponds to " + str(loopcount/toc) + " Hz.")
s1.close() #disconnect from vision network 

#### Postrun data processing ####
#np.save("RawCameraData", RawCameraData) #save for later processing. Access single camera outputs with RawCameraData=np.load("RawCameraData.npy") (x1, y1, a1, x2, y2, a2, timestamp) = RawCameraData[i]

#Start Plotting results
close(realtime_fig)

plt.figure()
plt.plot(pxRaw,pyRaw,'xr',label='measured')
plt.plot(pxFilt,pyFilt,'ob',label='kalman output')
plt.legend(loc=2)
plt.title("x-y Plane (top view)")
plt.xlabel('x')
plt.ylabel('y')

plt.figure()
plt.plot(timevec,pxRaw,'xr',label='measured')
plt.plot(timevec,pxFilt, 'ob', label='kalman output')
plt.title('x Position')
plt.xlabel('Time [s]')
plt.ylabel('x Position')
plt.legend(loc=2)

plt.figure()
plt.plot(timevec,pyRaw,'xr',label='measured')
plt.plot(timevec,pyFilt, 'ob', label='kalman output')
plt.title('y Position')
plt.xlabel('Time [s]')
plt.ylabel('y Position')
plt.legend(loc=2)

plt.figure()
plt.plot(timevec,pzRaw,'xr',label='measured')
plt.plot(timevec,pzFilt, 'ob', label='kalman output')
plt.title('z Position')
plt.xlabel('Time [s]')
plt.ylabel('z Position')
plt.legend(loc=2)

goalcorners = np.matrix([[pxGoalie,-goalwidth/2,0],[pxGoalie,-goalwidth/2,goalheight],[pxGoalie,goalwidth/2,goalheight],[pxGoalie,goalwidth/2,0]])
fig3d = plt.figure(5)
ax = fig3d.add_subplot(111, projection='3d',)
ax.scatter(pxRaw,pyRaw,pzRaw, c='r', marker='x', label='measured')
ax.scatter(pxFilt,pyFilt,pzFilt, c='b', marker='o', label='kalman output')
ax.scatter(0,0,7, c='w', marker='x')
xx, yy = np.meshgrid(range(0,4,3), range(-3,4,6))
ax.plot_surface(xx,yy,0, color='lightgreen')
ax.plot(goalcorners[:,0],goalcorners[:,1],np.squeeze(np.asarray(goalcorners[:,2])), 'k')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
#ax.legend(loc=2)
#TODO: animated plot

plt.show()
#End plotting results

raw_input('Please press any key to quit')
plt.close('all')

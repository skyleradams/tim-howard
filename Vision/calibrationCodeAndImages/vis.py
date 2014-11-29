import cv2
import numpy as np
import sys, time, math

data = []

for j in range(1,6):
	for i in range(1,12):
		img = cv2.imread('calibration/'+str(12*i)+'x'+str(16*j)+'.png',1)
		if img==None:
			print "cannot open ",filename
		else:
			#img[:,:,1] = 0
			gimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
			circles = cv2.HoughCircles(gimg,cv2.cv.CV_HOUGH_GRADIENT,1,
				minDist = 1000,param1=250,param2=7,minRadius=12,maxRadius=18)
			for k in circles[0,:]:
				# draw the outer circle
				cv2.circle(img,(k[0],k[1]),k[2],(0,255,0),2)
				# draw the center of the circle
				cv2.circle(img,(k[0],k[1]),2,(0,0,255),3)
			circles = np.uint16(np.around(circles))
			circles = circles[0][0]
			data.append([circles[0], 472 - circles[1], 12*i, 16*j])
			cv2.imshow('detected',img)
			cv2.waitKey(0)
			cv2.imwrite('detected/'+str(12*i)+'x'+str(16*j)+'.png',img)

myFile=open('calibrationData.txt', 'w')
[myFile.write(str(x)+'\n') for x in data]
myFile.close()
cv2.destroyAllWindows()

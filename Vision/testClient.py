"""
2.12 Vision Client Test Code
MIT 2.12 Intro To Robotics 2014
Daniel J. Gonzalez - dgonz@mit.edu
"""

########  Imports and functions required at the top of your main python file.  ####
import sys,socket,time,struct,signal
def signal_handler(signal, frame):
    global s1
    print('Closing Connection...')
    s1.close()
    sys.exit(0)

def getVals(sObj):
    sObj.send('r')
    raw = sObj.recv(16)
    x1 = struct.unpack('>H',raw[0:2])[0]
    y1 = struct.unpack('>H',raw[2:4])[0]
    a1 = struct.unpack('>H',raw[4:6])[0]
    x2 = struct.unpack('>H',raw[6:8])[0]
    y2 = struct.unpack('>H',raw[8:10])[0]
    a2 = struct.unpack('>H',raw[10:12])[0]
    timestamp = struct.unpack('>f',raw[12:16])[0]
    return (x1, y1, a1, x2, y2, a2, timestamp)
    
########  Set up TCP/IP Connection ####
#serverIP = '192.168.1.212'  #Use with the 2.12 Servers
serverIP = 'localhost'      #Use for loopback testing on your own computer
s1=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
try:
    s1.connect((serverIP,2121))
    print 'Connecting...'
except socket.error:
    print('connection error')
    sys.exit(0)

########  Run your main loop code here. Here I'm just looping 100 times.  ####
tic = time.time() #For timing purposes

N = 100
for n in range(0,N):
    (x1, y1, a1, x2, y2, a2, timestamp) = getVals(s1)

toc = time.time()-tic #For timing purposes
print 'average time per read: ',str(toc/N) #Avg time per read: 

########  Close the connection when you end the program.  ####
print('Closing Connection...')
s1.close()
print (x1, y1, a1, x2, y2, a2, timestamp)
#Print latest result to prove that we have the proper
# For example: (1000, 2000, 3000, 4000, 5000, 6000, 88.26200103759766)

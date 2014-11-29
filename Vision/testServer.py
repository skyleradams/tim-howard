"""
2.12 Vison Local Server Test
MIT 2.12 Intro To Robotics 2014
Daniel J. Gonzalez - dgonz@mit.edu
"""

serverIP = 'localhost' #Use if loopback testing on your own computer
#serverIP = '192.168.1.212' #Use if this code is running over a 2.12 Server

################# DO NOT EDIT ANYTHING BELOW #########################
import threading,SocketServer,time
import signal
import sys
import struct
tStart = time.time()
timestamp = tStart
n = 0

def signal_handler(signal, frame):
        print('Closing...')
        server.socket.close()
        sys.exit(0)
        
signal.signal(signal.SIGINT, signal_handler)

class requestHandler(SocketServer.StreamRequestHandler):
    def handle(self):
        requestForUpdate=self.request.recv(256)
        print(self.client_address)
        while requestForUpdate!='':
            state = [1000, 2000, 3000, 4000, 5000, 6000]
            data1 = ''.join([struct.pack('>H',x) for x in state])
            data2 = struct.pack('>f',timestamp)
            self.wfile.write(data1+data2)
            requestForUpdate=self.request.recv(256)
        print('client disconnect')

class broadcastServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    pass

if __name__ == '__main__':

    server=broadcastServer((serverIP,2121),requestHandler)
    t = threading.Thread(target=server.serve_forever)
    t.daemon=True
    t.start()
    print('server start')
    n=0
    while n<=10000:
        timestamp = time.time() - tStart
        n+=1
        time.sleep(.001)
    server.socket.close()

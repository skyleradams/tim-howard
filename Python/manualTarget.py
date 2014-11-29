import serial, time
import goalieFunctions
import platform

#### Setup USB Connection to OpenCM ####
if platform.system() == 'Darwin': # mac os
    ser = serial.Serial(
    port='/dev/tty.usbmodem1411',
    baudrate=9600,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS,
    timeout=None
    )
else: # linux
    ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=9600,
        parity=serial.PARITY_ODD,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.SEVENBITS,
        timeout=None
    )
ser.isOpen()


loopcount = 0
while True:

	
	target = input("Enter [t_goal, y_goal, z_goal]: ")
	t_goal = target[0]
	y_goal = target[1]
	z_goal = target[2]
	

	ser.write(goalieFunctions.packInteger(t_goal*100)+goalieFunctions.packInteger(y_goal*100)+goalieFunctions.packInteger(z_goal*100))

	
	out = ''
        # let's wait before reading output (give device time to answer)
        time.sleep(0.1)
        while ser.inWaiting() > 0: #.inWaiting(): returns how many chars are in buffer to be read
            out += ser.read(1)

        if out != '':
            print ">>" + out

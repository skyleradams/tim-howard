# measurements in inches
ball_radius = 3
goal_top = 50
goal_width = 58
goal_half = 29

angle_threshold = .1



class L_params(object):
	horizontal_offset = 14.5
	vertical_offset = 18.5
	min_y = ball_radius - vertical_offset+4 # in robot coords
	max_y = goal_top - vertical_offset
	min_x = -14.5
	max_x = 14.0
	l1 = 11
	l2 = 11
	shoulder_offset = 0
	elbow_offset = -50
	angle_threshold = angle_threshold


class R_params(object):
	horizontal_offset = 43.5
	vertical_offset = 18.5
	min_y = ball_radius - vertical_offset+4 # in robot coords
	max_y = goal_top - vertical_offset
	min_x = -14.0
	max_x = 14.5
	l1 = 11
	l2 = 11
	shoulder_offset = 0
	elbow_offset = 0
	angle_threshold = angle_threshold

left_arm = L_params()
right_arm = R_params()

windows_port = "COM8"
unix_port = "/dev/tty.usbserial-A4012933"
ubuntu_port = "/dev/ttyUSB0"

num_servos = 2
servo_speed = 1000
baudrate = 400000


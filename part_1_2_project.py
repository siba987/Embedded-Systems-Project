import tuio
import socket
import time
import math

TCP_IP = '192.168.1.1'
TCP_PORT = 2001
BUFFER_SIZE = 1024
forward = b'\xff\0\x01\0\xff'
bckwd = b'\xff\0\x02\0\xff'
stop = b'\xff\0\x00\0\xff'
rot_r = b'\xFF\x00\x03\x00\xFF'
rot_l = b'\xFF\x00\x04\x00\xFF'
save_cam_angle = b'\xFF\x32\x00\x00\xFF'
reset_cam_angle = b'\xFF\x33\x00\x00\xFF'

def new_cnct(): #setting up a new connection
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))
	return s

def move_fwd_2(): #function to move forward with own socket connection
	skt = new_cnct()
	skt.send(b'\xFF\x02\x01\x18\xFF')
	skt.send(b'\xFF\x02\x02\x18\xFF')
	skt.send(forward)
	skt.close()

def move_stp_2(): #function to stop with own socket connection
	skt = new_cnct()
	skt.send(stop)
	skt.close()

def rotate_r_3(delay): #function to rotate right incrementally (with delay) with own socket connection
	skt = new_cnct()
	skt.send(rot_r)
	time.sleep(delay)
	skt.send(stop)
	skt.close()

def rotate_l_3(delay): #function to rotate left incrementally (with delay) with own socket connection
	skt = new_cnct()
	skt.send(rot_l)
	time.sleep(delay)
	skt.send(stop)
	skt.close()

def check_direction(agl_r, agl_s, is_right): #returning final angle robot must reach
	if(is_right):
		final_angle = agl_s - agl_r #rotating right results in a decrease in angle value
	else:
		final_angle = agl_s + agl_r #rotating left results in an increase in angle value
	return final_angle

def check_overflow(agl_r, agl_s, is_right): #altering angle in case of need of wraparound
	if(is_right):
		dif = agl_s - agl_r #rotating right results in a decrease in angle value
		if(dif < 0):
			agl_s += 360 
	else:
		sum_agl = agl_s + agl_r #rotating left results in an increase in angle value
		if(sum_agl > 360):
			agl_s -= 360
	return agl_s

def check_case(sx, sy, ex, ey): #function to determine position of end fm relative to robot and calculate theta
	#pass in start and end coordinates

	theta = math.degrees(math.atan(abs(ey-sy)/abs(ex-sx)))

	if(sx>ex):
		if(sy<ey):
			case = 'nw'
		else:
			case = 'sw'
	else:
		if(sy<ey):
			case = 'ne'
		else:
			case = 'se'

	return theta, case

def det_rot_angle(agl_s, theta, case):
	#agl_s corresponds to starting angle of the robot 
	#rot_angle is the angle the robot must rotate by to reach correct axis 

	if(case == "nw"):
		if((agl_s > (180-theta)) and (agl_s < 360)):
			rot_angle = agl_s - 180 + theta
			is_right = 1
		else:
			rot_angle = 180 - agl_s - theta
			is_right = 0

	elif(case == "se"):
		if(agl_s > 0) and (agl_s <= 180):
			rot_angle = agl_s + theta
			# rot_angle = -theta
			is_right = 1
		elif(agl_s > 180) and (agl_s <= (360 - theta)):
			rot_angle = 360 - agl_s - theta
			is_right = 0
		else:
			rot_angle = agl_s - 360 + theta
			is_right = 1

	elif(case == "ne"):
		if(agl_s < theta):
			rot_angle = theta - agl_s
			is_right = 0

		elif((agl_s > theta) and (agl_s < 270)):
			rot_angle = agl_s - theta
			is_right = 1

		elif(agl_s > 270) and (agl_s < 360):
			rot_angle = 360 - agl_s + theta
			is_right = 0

	elif(case == "sw"):
		if(agl_s < (180 + theta)):
			rot_angle = 180 - agl_s + theta
			is_right = 0
		elif(agl_s >= (180 + theta)):
			rot_angle = agl_s - 180 - theta
			is_right = 1

	agl_s = check_overflow(rot_angle, agl_s, is_right) #accounting for overflow of >360 or <0, change starting angle value if necessary
	rot_angle = check_direction(rot_angle, agl_s, is_right)  #calculating destination angle

	return rot_angle, is_right

def det_fm_info_end(): #function to retrieve information of goal/end point
	tracking = tuio.Tracking()
	while 1:
			tracking.update()
			for obj in tracking.objects():
					if(obj.id == 3):
						print obj
						tracking.stop()
						return obj.angle, obj.xpos, obj.ypos

def det_fm_info_start(): #function to retrieve initial information of robot's starting point
	tracking = tuio.Tracking()
	while 1:
			tracking.update()
			for obj in tracking.objects():
					if(obj.id == 0):
						print obj
						tracking.stop()
						return obj.angle, obj.xpos, obj.ypos 

def check_xy(xr, yr, goal_x, goal_y, xy_range): #checking if x and y position of robot is within specified error range  
	
	if(xr <= (goal_x + xy_range)) and (xr >= (goal_x - xy_range)):
		if(yr <= (goal_y + xy_range)) and (yr >= (goal_y - xy_range)):
			return True 
	return False  	

def main():

	mode = int(input("Choose your mode:\n1: Move to point B\n2: Follow another robot\n"))

	while 1:

		ae, xe, ye = det_fm_info_end() #must continually update robot position as it moves
		ar, xr, yr = det_fm_info_start()
		theta, case = check_case(xr, yr, xe, ye) 
		agl, is_right = det_rot_angle(ar, theta, case)

		if(mode == 1): #mode corresponding to moving to fixed point
			agl_range = 6  
			delay = 0.07
			xy_range = 0.07
		else: #mode corresponding to leader follower
			agl_range = 10 #angle and xy ranges are made larger as precision less important than ensuring follower reaches leader more quickly
			delay = 0.15
			xy_range = 0.15 #xy range set to be larger to avoid crashing of robots 

		if(((ar < (agl+agl_range)) and (ar > (agl-agl_range))) and not(check_xy(xr, yr, xe, ye, xy_range))):
			move_fwd_2() #moves forward if within angle range but not x/y range

		elif(not((ar < (agl+agl_range)) and (ar > (agl-agl_range)))):
			#rotates in appropriate direction depending on assigned direction
			if(is_right):
				rotate_r_3(delay)
			else:
				rotate_l_3(delay)
		else:
			move_stp_2()

			if(mode == 1):
				break #while loop is broken only if moving to fixed position, would not expect further movement
			
main()

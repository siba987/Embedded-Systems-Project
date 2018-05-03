import tuio
import socket
import time
import math
import datetime

#hex format of commands to be sent to control robot using TCP protocol
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

def set_speed_high(): #function for setting a relatively higher speed 
	skt = new_cnct()
	skt.send(b'\xFF\x02\x01\x12\xFF')
	skt.send(b'\xFF\x02\x02\x12\xFF')
	# skt.send(forward)
	skt.close()

def set_speed_med(): #function for medium speed
	skt = new_cnct()
	skt.send(b'\xFF\x02\x01\x0f\xFF')
	skt.send(b'\xFF\x02\x02\x0f\xFF')
	skt.close()

def set_speed_low(): #function for lower speed
	skt = new_cnct()
	skt.send(b'\xFF\x02\x01\x0a\xFF')
	skt.send(b'\xFF\x02\x02\x0a\xFF')
	skt.close()

def move_fwd_2(): #function to move forward with own socket connection
	skt = new_cnct()
	skt.send(forward)
	skt.close()

def move_fwd(s): #function which when passed socket object only sends forward hex command
	s.send(forward)

def move_stp(s): #function which when passed socket object only sends stop hex command
	s.send(stop)

def move_stp_2(): #function to stop with own socket connection
	skt = new_cnct()
	skt.send(stop)
	skt.close()

def rotate_r_3(): #function to rotate right incrementally (with delay) with own socket connection
	skt = new_cnct()
	skt.send(rot_r)
	time.sleep(0.03)
	skt.send(stop)
	skt.close()

def rotate_l_3(): #function to rotate left incrementally (with delay) with own socket connection
	skt = new_cnct()
	skt.send(rot_l)
	time.sleep(0.03)
	skt.send(stop)
	skt.close()

def check_direction(agl_r, agl_s, is_right): #returning final angle robot must reach
	if(is_right):
		final_angle = agl_s - agl_r
	else:
		final_angle = agl_s + agl_r
	return final_angle

def check_overflow(agl_r, agl_s, is_right):
	if(is_right):
		dif = agl_s - agl_r 
		if(dif < 0):
			agl_s += 360 
	else:
		sum_agl = agl_s + agl_r
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
	rot_angle = check_direction(rot_angle, agl_s, is_right) #returning destination angle

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
						# print obj
						tracking.stop()
						return obj.angle, obj.xpos, obj.ypos 

def check_xy(xr, yr, goal_x, goal_y, xy_range): #checking if x and y position of robot is within specified error range  
	if(xr <= (goal_x + xy_range)) and (xr >= (goal_x - xy_range)):
		if(yr <= (goal_y + xy_range)) and (yr >= (goal_y - xy_range)):
			return True 
	return False  	

def det_next_pos(idx): #keeping track of destination x and y coordinates
	x_points = [0.85, 0.83, 0.77, 0.68, 0.56, 0.44, 0.32, 0.23, 0.17, 0.15]
	y_points = [0.5, 0.72, 0.84, 0.8, 0.62, 0.38, 0.2, 0.16, 0.28, 0.5]

	return x_points[idx], y_points[idx]
	
def main():

	move_stp_2()
	set_speed_med() #setting speed to 0x0f
	agl_range = 6 #setting angle range to be acceptably large to minimize excessive error correcting
	xy_range = 0.03

	for i in range(1, 10): #loops for as many points as there are to reach 
		if(i==9):
			set_speed_low()
			xy_range = 0.06
		elif(i==3) or (i==4) or (i==5) or (i==6):
			set_speed_high()

		ar, xr, yr = det_fm_info_start()  #getting intitial robot information
		xn, yn = det_next_pos(i) #determining next position information
		theta, case = check_case(xr, yr, xn, yn) #retrieving theta and scenario information
		agl, is_right = det_rot_angle(ar, theta, case) #determining final angle robot must rotate to and in which direction
		
		end = False

		while end == False:  #while loop is broken once next position is reached
			ar, xr, yr = det_fm_info_start() #information of robot is continually updated 
			theta, case = check_case(xr, yr, xn, yn)
			agl, is_right = det_rot_angle(ar, theta, case)

			if(((ar < (agl+agl_range)) and (ar > (agl-agl_range))) and not(check_xy(xr, yr, xn, yn, xy_range))):
				move_fwd_2() #moves forward if within angle range but not x/y range
			elif(not((ar < (agl+agl_range)) and (ar > (agl-agl_range)))):
				#rotates in appropriate direction depending on assigned direction
				if(is_right):
					rotate_r_3()
				else:
					rotate_l_3()
			else:
				ts = time.time() #retrieving time at which the robot stopped at this position
				st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S') #converting timestamp to readable form

				print(xr, yr, xn, yn, st)
				move_stp_2()

				end = True

main()

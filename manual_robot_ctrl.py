#!/usr/bin/env python
 
import socket
import time

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

def move_fwd(skt): #function to move forward
  skt.send(forward)

def move_bckwd(skt): #function to move backwards
  skt.send(bckwd)

def move_stp(skt): #function to stop
  skt.send(stop)

def rotate_r(skt): #function to rotate right
  skt.send(rot_r)

def rotate_l(skt): #function to rotate left
  skt.send(rot_l)

def set_right_speed(skt): #changing right motor speed
  speed_no = int(input("Enter speed from 0-100:"))
  speed_no_hex = str(hex(speed_no))
  if(speed_no < 16): #ensuring single digit hex values are appropriate length
    speed_msg_str = 'FF02010' + speed_no_hex[2:len(speed_no_hex)] + 'FF'
  else:
    speed_msg_str = 'FF0201' + speed_no_hex[2:len(speed_no_hex)] + 'FF'
  speed_msg_bytes = bytes.fromhex(speed_msg_str)
  skt.send(speed_msg_bytes)

def set_left_speed(skt): #changing left motor speed
  speed_no = int(input("Enter speed from 0-100:"))
  speed_no_hex = str(hex(speed_no))
  if(speed_no < 16): #ensuring single digit hex values are appropriate length
    speed_msg_str = 'FF02020' + speed_no_hex[2:len(speed_no_hex)] + 'FF'
  else:
    speed_msg_str = 'FF0202' + speed_no_hex[2:len(speed_no_hex)] + 'FF'
  speed_msg_bytes = bytes.fromhex(speed_msg_str)
  skt.send(speed_msg_bytes)

def rot_cam_LR(skt): #rotate camera from left to right
    angle_no = int(input("Enter angle from 0-180:"))
    angle_no_hex = str(hex(angle_no))
    angle_msg_str = 'FF0107' + angle_no_hex[2:len(angle_no_hex)] + 'FF'
    angle_msg_bytes = bytes.fromhex(angle_msg_str)
    skt.send(angle_msg_bytes)

def rot_cam_UD(skt): #move camera up and down 
    angle_no = int(input("Enter angle from 0-180:"))
    angle_no_hex = str(hex(angle_no))
    angle_msg_str = 'FF0108' + angle_no_hex[2:len(angle_no_hex)] + 'FF'
    angle_msg_bytes = bytes.fromhex(angle_msg_str)
    skt.send(angle_msg_bytes)

def request_cmnd(socket): #continually asks for user input
  end = False

  while(end == False):
    cmnd = int(input("Enter command:\nForward: 1\nBackward: 2\nStop: 3\nRotate right: 4\nRotate left: 5\nChange right motor speed: 6\nChange left motor speed: 7\nClose connection: 0\n"))
    if(cmnd == 1):
      move_fwd(socket)
    elif(cmnd == 2):
      move_bckwd(socket)
    elif(cmnd == 3):
      move_stp(socket)
    elif(cmnd == 4):
      rotate_r(socket)
    elif(cmnd == 5):
      rotate_l(socket)
    elif(cmnd == 6):
      set_right_speed(socket)
    elif(cmnd == 7):
      set_left_speed(socket)
    elif(cmnd == 0):
        end = True

def main():

  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.connect((TCP_IP, TCP_PORT))
  request_cmnd(s)
  s.close()

main() 

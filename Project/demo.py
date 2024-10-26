#!/usr/bin/python3

# Using Dynamixel library from https://gitlab.control.lth.se/anders_blomdell/dynamixel
from dynamixel.model.ax_12a import AX_12A
from dynamixel.model.ax_18a import AX_18A
import dynamixel.channel

import time
import math
import numpy as np
from functions import *

pick_theta0 = 21
place_theta0 = -23

scoop1_theta1 = 30
scoop1_theta2 = -100

scoop2_theta1 = 5
scoop2_theta2 = -51

drop_theta1 = 36
drop_theta2 = -99

up_theta1 = 80

home = [172.5, 0, 41.5] # thetas = [0, 0, 0]

list = [[pick_theta0, 0, 0], 
		[pick_theta0, scoop1_theta1, scoop1_theta2],
		[pick_theta0, scoop2_theta1, scoop2_theta2],
		[pick_theta0, 0, 0],
		[place_theta0, 0, 0],
		[place_theta0, drop_theta1, drop_theta2],
		[place_theta0, up_theta1, drop_theta2]]

if __name__ == '__main__':
	channel = dynamixel.channel.Channel(speed=1000000, device='/dev/ttyUSB0')
	# Define specific joint servos
	servos = [AX_18A(channel, 1), AX_18A(channel, 2), AX_12A(channel, 3)]

	# Set parameters for angle limit and compliance margin
	for s in servos:
		s.torque_enable.write(0)
		# print(s.model_number.read(), s.id.read())
		time.sleep(0.1)
		s.cw_angle_limit.write(0)
		s.ccw_angle_limit.write(1023)
		s.cw_compliance_margin.write(1)
		s.ccw_compliance_margin.write(1)
		s.torque_enable.write(1)
	
	moveJ(servos, home, vel=500)
	input()
	
	for thetas in list:
		raw_move(servos, thetas, vel=500)
		input()
		# time.sleep(3.0)
	
	moveJ(servos, home, vel=500)
	
	input("Press enter to disable servos and finish")

	# Disables the servos
	for s in servos:
		s.torque_enable.write(0)

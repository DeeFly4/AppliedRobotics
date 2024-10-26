#!/usr/bin/python3

# Using Dynamixel library from https://gitlab.control.lth.se/anders_blomdell/dynamixel
from dynamixel.model.ax_12a import AX_12A
from dynamixel.model.ax_18a import AX_18A
import dynamixel.channel

import time
import math
import numpy as np
from functions import *

if __name__ == '__main__':
	channel = dynamixel.channel.Channel(speed=1000000, device='/dev/ttyUSB0')
	# Define specific joint servos
	servos = [AX_18A(channel, 1), AX_18A(channel, 2), AX_12A(channel, 3)]

	# Set parameters for angle limit and compliance margin
	for s in servos:
		s.torque_enable.write(0)
		#print(s.model_number.read(), s.id.read())
		time.sleep(0.1)
		s.cw_angle_limit.write(0)
		s.ccw_angle_limit.write(1023)
		s.cw_compliance_margin.write(1)
		s.ccw_compliance_margin.write(1)
		s.torque_enable.write(1)

	print("Enter starting position")
	x = float(input("x: "))
	y = float(input("y: "))
	z = float(input("z: "))
	
	start_pos = [x, y, z]
	
	moveJ(servos, start_pos)
	
	print("Enter target position")
	x = float(input("x: "))
	y = float(input("y: "))
	z = float(input("z: "))
	
	target_pos = [x, y, z]
	
	moveL(servos, target_pos)
	
	input("Press enter to disable servos and finish")

	# Disables the servos
	for s in servos:
		s.torque_enable.write(0)

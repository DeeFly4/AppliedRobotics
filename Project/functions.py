import math
import time
import numpy as np

# Default moving speed
move_vel = 100

# Link lengths
a = [0, 67.5, 105]

# Offset
d0 = 41.5

def int2rad(servo_positions):
	thetas = np.empty([3,], dtype=float)
	
	thetas[0] = (servo_positions[0] - 512) / 3.45
	thetas[1] = (823 - servo_positions[1]) / 3.45
	thetas[2] = (512 - servo_positions[2]) / 3.45
	
	return np.radians(thetas)

def deg2int(thetas):
	servo_positions = np.empty([3,], dtype=int)
	
	servo_positions[0] = int(512 + thetas[0]	* 3.45)
	servo_positions[1] = int(823 - thetas[1] * 3.45)
	servo_positions[2] = int(512 - thetas[2]	* 3.45)
    
	return servo_positions

def inv_kin(target_pos):
	x = target_pos[0]
	y = target_pos[1]
	z = target_pos[2]
	
	# Calculate the height of the target
	h = z - d0
	
	# Calculate the distance fromthe origin  to the target
	c = math.sqrt(x**2 + y**2 + h**2)
	
	# Check if the target is reachable
	if abs(a[1] - a[2]) <= c <= (a[1] + a[2]):  # define the constraints for the target position 
		# Calculate the angle for the third joint
		cos_theta2 = (c**2 - a[1]**2 - a[2]**2) / (2 * a[1] * a[2])
		if cos_theta2 < -1 or cos_theta2 > 1:
			raise ValueError("Target is out of reach due to domain error in cos_theta2")
		else:
			# Flip the sign due to axis orientation
			theta2 = -math.acos(cos_theta2)
			
			# Calculate the angle for the second joint
			r = math.sqrt(x**2 + y**2)
			alpha = math.atan2(h, r)
			beta = math.acos((a[1]**2 + c**2 - a[2]**2) / (2 * a[1] * c))
			theta1 = alpha + beta
			cos_theta1 = math.cos(theta2)
			sin_theta1 = math.sin(theta2)

			# Calculate the angle for the first joint
			theta0 = math.atan2(y, x)

			return np.degrees(np.array([theta0, theta1, theta2]))
	else:
		raise ValueError("Target is out of reach")

def forward_kin(thetas):
	#  Transformation matrices
	A0 = np.array([
		[math.cos(thetas[0]), 0, math.sin(thetas[0]), 0],
		[math.sin(thetas[0]), 0, -math.cos(thetas[0]), 0],
		[0, 1, 0, d0],
		[0, 0, 0, 1]
		])
		
	A1 = np.array([
		[math.cos(thetas[1]), -math.sin(thetas[1]), 0, a[1]*math.cos(thetas[1])],
		[math.sin(thetas[1]), math.cos(thetas[1]), 0, a[1]*math.sin(thetas[1])],
		[0, 0, 1, 0],
		[0, 0, 0, 1]
		])
		
	A2 = np.array([
		[math.cos(thetas[2]), -math.sin(thetas[2]), 0, a[2]*math.cos(thetas[2])],
		[math.sin(thetas[2]), math.cos(thetas[2]), 0, a[2]*math.sin(thetas[2])],
		[0, 0, 1, 0],
		[0, 0, 0, 1]
		])

	A = A0@A1@A2
	
	return A[0:3, 3]

def moveJ(servos, target_pos, vel = move_vel):
	# Calculates the required joint angles with inverse kinematics
	try:
		thetas = inv_kin(target_pos)
	except ValueError as err:
		print(err)
		return
	
	# Converts the joint angles from degrees to servo positions betweeen 0-1023
	servo_positions = deg2int(thetas)
	
	# Sets the moving speed and then moves all the joints
	for i in range(3):
		servos[i].moving_speed.write(move_vel)
		servos[i].goal_position.write(servo_positions[i])
		# time.sleep(3.0) # If you wanna move one joint at a time

def raw_move(servos, thetas, vel=move_vel):	
	# Converts the joint angles from degrees to servo positions betweeen 0-1023
	servo_positions = deg2int(thetas)
	
	# Sets the moving speed and then moves all the joints
	for i in range(3):
		servos[i].moving_speed.write(move_vel)
		servos[i].goal_position.write(servo_positions[i])
	
def moveL(servos, target_pos, vel=move_vel, spacing=5):
	# Reads the current servo positions
	servo_positions = [s.present_position.read() for s in servos]

	# Convert to radians
	thetas = int2rad(servo_positions)

	# Calculate position in space using forward kinematics
	current_pos = forward_kin(thetas)

	# Find the joint angles needed for all intermediate steps
	xDiff = target_pos[0] - current_pos[0]
	yDiff = target_pos[1] - current_pos[1]
	zDiff = target_pos[2] - current_pos[2]

	dist = np.sqrt(xDiff**2 + yDiff**2 + zDiff**2)
	n_steps = int(np.round(dist/spacing))
	
	xStep = xDiff / n_steps
	yStep = yDiff / n_steps
	zStep = zDiff / n_steps

	middle_thetas = np.empty([n_steps + 1, 3], dtype=float)

	for i in range(n_steps):
		x = current_pos[0] + i*xStep
		y = current_pos[1] + i*yStep
		z = current_pos[2] + i*zStep
		
		try:
			middle_thetas[i] = inv_kin([x, y, z])
		except ValueError as err:
			print(err)
			return

	try:
		middle_thetas[-1] = inv_kin(target_pos)
	except ValueError as err:
		print(err)
		return

	# Loops through the list of intermediate steps
	for i in range(n_steps + 1):
		servo_positions = deg2int(middle_thetas[i])
		
		# Set the moving speed and then move all the joints
		for i in range(3):
			servos[i].moving_speed.write(move_vel)
			servos[i].goal_position.write(servo_positions[i])
		
		time.sleep(0.2) # Can be very short since the steps are small

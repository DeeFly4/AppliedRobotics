import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt
import math

Link0 = rtb.RevoluteDH(d=41.5, alpha=np.pi/2)
Link1 = rtb.RevoluteDH(a=67.5)
Link2 = rtb.RevoluteDH(a=105)

robot = rtb.DHRobot([Link0, Link1, Link2])

max_vel = 0.26

def inv_kin(target):
	x = target[0]
	y = target[1]
	z = target[2]
	
	h = z - Link0.d

	r = np.hypot(x, y)

	c = np.sqrt(x**2 + y**2 + h**2)

	# Calculate the angle for the first joint
	theta0 = math.atan2(y, x)
	
	if abs(Link1.a - Link2.a) <= c <= (Link1.a + Link2.a):  # define the constraints for the target position 
		# Calculate the angle for the third joint
		cos_theta2 = (c**2 - Link1.a**2 - Link2.a**2) / (2 * Link1.a * Link2.a)
		if cos_theta2 < -1 or cos_theta2 > 1:
			raise ValueError("Target is out of reach due to domain error in cos_theta2")
		else:
			# Flip the sign to get elbow-up configuration
			theta2 = -2*math.atan(np.sqrt((1 - cos_theta2) / (1 + cos_theta2)))
			
			alpha = math.atan2(h, r)
			beta = math.acos((Link1.a**2 + c**2 - Link2.a**2) / (2 * Link1.a * c))
			# theta1 is alpha+beta to get elbow-up configuration
			theta1 = alpha + beta

			return np.array([theta0, theta1, theta2])
	else:
		raise ValueError("Target is out of reach")

def ptraj(q0, qf, dt=0.05):
	diffs = qf - q0

	max_diff = np.max(np.abs(diffs))

	move_time = max_diff / max_vel

	speeds = np.array(diffs) / move_time

	t = np.linspace(0, move_time, num=int(move_time / dt))

	return q0 + np.array([speeds*time for time in t])

def interpolate(start_pos, end_pos, spacing=10):
	x = start_pos[0]
	y = start_pos[1]
	z = start_pos[2]

	x_diff = end_pos[0] - x
	y_diff = end_pos[1] - y
	z_diff = end_pos[2] - z
	
	dist = np.sqrt(x**2 + y**2 + z**2)
	n_steps = int(dist / spacing)
	
	x_step = x_diff / n_steps
	y_step = y_diff / n_steps
	z_step = z_diff / n_steps
	
	thetas = np.empty((n_steps+1, 3), dtype=float)
	mid_points = np.array(start_pos)
	
	for i in range(n_steps):
		try:
			thetas[i] = inv_kin([x, y, z])
		except ValueError as err:
			print(err)
			print(thetas)
			return

		x += x_step
		y += y_step
		z += z_step

		mid_points = np.vstack((mid_points, [x, y, z]))
	
	thetas[-1] = inv_kin(end_pos)

	return mid_points, thetas

def linear_traj(start, end, dt=0.05):
	mid_points, theta_targets = interpolate(start, end)
 
	theta0 = theta1 = theta2 = []
	
	for i in range(len(theta_targets) - 1):
		q0 = theta_targets[i]
		qf = theta_targets[i+1]

		traj = ptraj(q0, qf, dt)

		theta0 = np.append(theta0, traj[:, 0])
		theta1 = np.append(theta1, traj[:, 1])
		theta2 = np.append(theta2, traj[:, 2])

	return mid_points, np.array([theta0, theta1, theta2]).T

if __name__ == '__main__':
	start = [172.5, 0, 41.5]
	end = [0, 172.5, 41.5]

	mid_points, thetas = linear_traj(start, end)

	T_list = robot.fkine(thetas)

	pos = np.array([T.t for T in T_list])

	plt.plot(mid_points[:, 0], mid_points[:, 1], '.')
	plt.plot(pos[:, 0], pos[:, 1])
	plt.show()

	robot.plot(thetas, dt=0.05)
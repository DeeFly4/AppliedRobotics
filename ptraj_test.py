import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt
import math

Link1 = rtb.RevoluteDH(a=1)
Link2 = rtb.RevoluteDH(a=1)

robot = rtb.DHRobot([Link1, Link2])

max_vel = 0.26

start = [2, 0]
end = [0, 2]

def inv_kin(target):
	x = target[0]
	y = target[1]
	
	aplus = Link1.a**2 + Link2.a**2
	twoa1a2 = 2 * Link1.a * Link2.a
	
	r2 = x**2 + y**2
	
	theta2 = 2*math.atan(math.sqrt((twoa1a2 + aplus - r2) / (twoa1a2 - aplus + r2)))
	
	if theta2 < 0:
		theta2 = -theta2
	
	theta1 = math.atan2(y, x) - math.atan2(Link2.a * math.sin(theta2), Link1.a + Link2.a*math.cos(theta2))
	
	return np.array([theta1, theta2])

def ptraj(q0, qf):
	diffs = qf-q0

	max_diff = np.max(np.abs(diffs))

	speeds = max_vel * np.array([diffs[0]/max_diff, diffs[1]/max_diff])

	t = np.linspace(0, max_diff / max_vel)

	return q0 + np.array([[speeds[0]*time, speeds[1]*time] for time in t])

def interpolate(start_pos, end_pos, spacing=0.5):
	x = start_pos[0]
	y = start_pos[1]

	x_diff = end_pos[0] - x
	y_diff = end_pos[1] - y
	
	dist = np.hypot(x_diff, y_diff)
	n_steps = int(dist / spacing)
	
	x_step = x_diff / n_steps
	y_step = y_diff / n_steps
	
	thetas = np.empty((n_steps+1, 2), dtype=float)
	coords = np.array([x, y])
	
	for i in range(n_steps):
		try:
			thetas[i] = inv_kin([x, y])
		except ValueError as err:
			print(err)
			print(thetas)
			return

		x += x_step
		y += y_step

		coords = np.vstack((coords, [x, y]))
	
	thetas[-1] = inv_kin(end_pos)

	return coords, thetas

def linear_traj(start, end):
	coords, theta_targets = interpolate(start, end)
 
	theta1 = theta2 = []
	
	for i in range(len(theta_targets) - 1):
		q0 = theta_targets[i]
		qf = theta_targets[i+1]

		traj = ptraj(q0, qf)

		theta1 = np.append(theta1, traj[:, 0])
		theta2 = np.append(theta2, traj[:, 1])

	return coords, np.array([theta1, theta2]).T	

if __name__ == '__main__':
	coords, thetas = linear_traj(start, end)

	T_list = robot.fkine(thetas)

	pos = np.array([[T.t[0], T.t[1]] for T in T_list])

	plt.plot(coords[:, 0], coords[:, 1], '.')
	plt.plot(pos[:, 0], pos[:, 1])
	plt.show()

	robot.plot(thetas)
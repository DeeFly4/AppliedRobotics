import numpy as np
import math
import matplotlib.pyplot as plt

targets = [[1, 3],
		   [2, 1],
		   [1, 1],
		   [3, 2]]

def forward_kin(thetas):
	return np.array([2*(math.cos(thetas[0]) + math.cos(thetas[0] + thetas[1])), 2*(math.sin(thetas[0]) + math.sin(thetas[0] + thetas[1]))])

def inverse_kin(target_pos):
	x = target_pos[0]
	y = target_pos[1]
	
	r2 = x**2 + y**2
	
	theta2 = 2*math.atan(math.sqrt((16 - r2) / r2))
	
	if theta2 < 0:
		theta2 = -theta2

	theta1 = math.atan2(y,x) - math.atan2(2*math.sin(theta2), 2 + 2*math.cos(theta2))

	return np.array([theta1, theta2])

# Calculates the cubic parameters for a given movement
def cubic_parameters(q0, qf, t0, tf, v0=0, vf=0):
	T = np.array([[1, t0, t0**2, t0**3],
				  [0, 1, 2*t0, 3*t0**2],
				  [1, tf, tf**2, tf**3],
				  [0, 1, 2*tf, 3*tf**2]
				  ])
	
	y1 = np.array([q0[0], v0, qf[0], vf])
	y2 = np.array([q0[1], v0, qf[1], vf])
	
	a1 = np.linalg.solve(T, y1)
	a2 = np.linalg.solve(T, y2)
	
	return a1, a2

# Plots the cubic trajectory for a given set of cubic parameters, used in plot_traj() and plot_pos()
def trajectories(a1, a2, t0, tf):
	t = np.linspace(t0, tf)

	q1 = a1[0] + a1[1]*t + a1[2]*t**2 + a1[3]*t**3
	q1dot = a1[1] + 2*a1[2]*t + 3*a1[3]*t**2
	q2 = a2[0] + a2[1]*t + a2[2]*t**2 + a2[3]*t**3
	q2dot = a2[1] + 2*a2[2]*t + 3*a2[3]*t**2

	return q1, q2, q1dot, q2dot
	
# Plots the position of the end-effector in space over time
def plot_pos():
	q0 = [np.pi/2, 0, 0]

	pos_vec = [[0,4]]
	
	for i, pos in enumerate(targets):
		t = np.linspace(i, i+1, 25)

		qf = inverse_kin(pos)

		a1, a2 = cubic_parameters(q0, qf, i, i+1)

		q0 = qf

		q1, q2, _, _ = trajectories(a1, a2, i, i+1)

		pos_vec = np.vstack((pos_vec, np.array([forward_kin(thetas) for thetas in zip(q1,q2)])))

	plt.plot(pos_vec[:, 0], pos_vec[:, 1])

	for char, pos in zip(['A', 'B', 'C', 'D'], targets):
		plt.annotate(char, xy=(pos[0], pos[1]), xytext=(pos[0], pos[1]-.2))

	plt.xlabel('x', fontsize=14)
	plt.ylabel('y', fontsize=14)
	plt.title('Position of end-effector over time')

	plt.xlim(0, 4.5)
	plt.ylim(0, 4.5)

	plt.grid()
	plt.show()

# Plots the cubic trajectories for all movements, one new window for each movement
def plot_traj():
	q0 = [np.pi/2, 0, 0]
	
	q1_vec = q2_vec = q1dot_vec = q2dot_vec = []
 
	for i, pos in enumerate(targets):
		qf = inverse_kin(pos)

		a1, a2 = cubic_parameters(q0, qf, i, i+1)

		q1, q2, q1dot, q2dot = trajectories(a1, a2, i, i+1)

		q1_vec = np.append(q1_vec, q1)
		q2_vec = np.append(q2_vec, q2)
		q1dot_vec = np.append(q1dot_vec, q1dot)
		q2dot_vec = np.append(q2dot_vec, q2dot)

		q0 = qf

	t_vec = np.linspace(0, 4, num=len(q1_vec)) # They all have the same length

	fig, axs = plt.subplots(2, 2)
	axs[0, 0].plot(t_vec, q1_vec)
	axs[0, 1].plot(t_vec, q2_vec)
	axs[1, 0].plot(t_vec, q1dot_vec)
	axs[1, 1].plot(t_vec, q2dot_vec)

	axs[0, 0].set_ylabel('Joint angle (rad)', fontsize=12)
	axs[0, 1].set_ylabel('Joint angle (rad)', fontsize=12)
	axs[1, 0].set_ylabel('Joint velocity (rad/s)', fontsize=12)
	axs[1, 1].set_ylabel('Joint velocity (rad/s)', fontsize=12)

	axs[0, 0].set_title(r'$\theta_1$', fontsize=16)
	axs[0, 1].set_title(r'$\theta_2$', fontsize=16)

	for ax in axs.flat:
		ax.grid()
		ax.set_xlabel('Time (s)', fontsize=12)

	fig.set_size_inches(12, 12)

	fig.suptitle('Joint-interpolated trajectories', fontsize=18)

	plt.show()

if __name__ == '__main__':
	plot_pos()
	plot_traj()
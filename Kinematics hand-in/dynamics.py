import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt

def create_bot():
	L1 = rtb.RevoluteDH(d=0.352, a=0.07, alpha=np.pi/2)
	L2 = rtb.RevoluteDH(a=0.36, offset=np.pi/2)
	L3 = rtb.RevoluteDH(a=0.445, offset=-np.pi/2)

	L1.m = 0
	L2.m = 35
	L3.m = 25

	L2.r = [-L2.a/2, 0, 0]
	L3.r = [-L3.a/2, 0, 0]

	bot = rtb.DHRobot([L1, L2, L3], name='IRB140', manufacturer='ABB')

	bot.gravity = [0, 0, -9.81]

	return bot

def get_torques(bot, q=[0, 10, 40]):
	print(bot.gravload(q))

def plot_traj(bot, q0 = np.radians([10, 20, 30]), qf = np.radians([-10, 40, 10])):
	Q = rtb.jtraj(q0, qf, 100)

	t_vec = np.linspace(0, 4, num=len(Q.q))
	
	fig, axs = plt.subplots(2, 3)
	axs[0, 0].plot(t_vec, Q.q[:, 0])
	axs[0, 1].plot(t_vec, Q.q[:, 1])
	axs[0, 2].plot(t_vec, Q.q[:, 2])
	axs[1, 0].plot(t_vec, Q.qd[:, 0])
	axs[1, 1].plot(t_vec, Q.qd[:, 1])
	axs[1, 2].plot(t_vec, Q.qd[:, 2])

	axs[0, 0].set_ylabel('Joint angle (rad)', fontsize=12)
	axs[0, 1].set_ylabel('Joint angle (rad)', fontsize=12)
	axs[0, 2].set_ylabel('Joint angle (rad)', fontsize=12)
	axs[1, 0].set_ylabel('Joint velocity (rad/s)', fontsize=12)
	axs[1, 1].set_ylabel('Joint velocity (rad/s)', fontsize=12)
	axs[1, 2].set_ylabel('Joint velocity (rad/s)', fontsize=12)

	axs[0, 0].set_title(r'$\theta_0$', fontsize=16)
	axs[0, 1].set_title(r'$\theta_1$', fontsize=16)
	axs[0, 2].set_title(r'$\theta_2$', fontsize=16)

	for ax in axs.flat:
		ax.grid()
		ax.set_xlabel('Time (s)', fontsize=12)

	fig.suptitle('Joint-interpolated trajectories', fontsize=18)
	
	plt.show()

	bot.plot(Q.q, block=True)

def plot_freefall(bot, q0=np.zeros((3,))):
	Q = bot.fdyn(1, q0)
 
	T_list = bot.fkine(Q.q)
 
	pos = np.array([[T.t[0], T.t[2]] for T in T_list])

	plt.plot(pos[:, 0], pos[:, 1])
	plt.show()

	bot.plot(Q.q, block=True)

if __name__ == '__main__':
	irb = create_bot()

	# get_torques(irb)
	# plot_traj(irb)
	# plot_freefall(irb)
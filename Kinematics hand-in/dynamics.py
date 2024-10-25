import roboticstoolbox as rtb
import numpy as np

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

def get_torques(bot, q):
	print(bot.gravload(q))

def plot_traj(bot, q0 = np.radians([10, 20, 30]), qf = np.radians([-10, 40, 10])):
	Q = rtb.jtraj(q0, qf, 100)
	
	bot.plot(Q.q, block=True)

def plot_freefall(bot, q0=np.zeros((3,))):
	Q = bot.fdyn(2, q0)

	bot.plot(Q.q, block=True)

if __name__ == '__main__':
	irb = create_bot()

	get_torques()
	plot_traj()
	plot_freefall()
import math
import numpy as np

#   Theta angles for the joints
theta = [0,0,0] 
for i in range(3):
	theta[i] = math.radians(float(input(f"Enter the angle for the first joint {i}: ")))

#   Offset [mm]
d0 = 41.5
d1 = 0
d2 = 0

#   Link lengths [mm]
a0 = 0
a1 = 67.5
# length of the spoon
a2 = 105

# link twist
alpha0 = math.pi/2
alpha1 = 0
alpha2 = 0

#  Transformation matrix
A0 = np.array([
	[math.cos(theta[0]), 0, math.sin(theta[0]), 0],
	[math.sin(theta[0]), 0, -math.cos(theta[0]), 0],
	[0, 1, 0, d0],
	[0, 0, 0, 1]
	])                                             # A0 = alpha0 => [0 0 0; 0 0 1; 0 -1 0]
	
A1 = np.array([
	[math.cos(theta[1]), -math.sin(theta[1]), 0, a1*math.cos(theta[1])],
	[math.sin(theta[1]), math.cos(theta[1]), 0, a1*math.sin(theta[1])],
	[0, 0, 1, 0],
	[0, 0, 0, 1]
	])
	
A2 = np.array([
	[math.cos(theta[2]), -math.sin(theta[2]), 0, a2*math.cos(theta[2])],
	[math.sin(theta[2]), math.cos(theta[2]), 0, a2*math.sin(theta[2])],
	[0, 0, 1, 0],
	[0, 0, 0, 1]
	])

A = A0@A1@A2

print(A)

	

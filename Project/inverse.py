import math

# Link lengths
a = [41.5, 67.5, 105]

# Target position 
x = 150
y = -80
z = 41.5

h = z - a[0]  # Calculate the height of the target

# Calculate the distance from the origin to the target
c = math.sqrt(x**2 + y**2 + h**2)  # c=sqrt(x^2 + y^2 + z^2)
c1 = math.sqrt(x**2 + y**2 + z**2)

# Check if the target is reachable
if abs(a[1] - a[2]) <= c <= (a[1] + a[2]):  # define the constraints for the target position 
    # Calculate the angle for the third joint
    cos_theta3 = (c**2 - a[1]**2 - a[2]**2) / (2 * a[1] * a[2])
    if cos_theta3 < -1 or cos_theta3 > 1:
        print("Target is out of reach due to domain error in cos_theta3")
    else:
        #sin_theta3 = math.sqrt(1 - cos_theta3**2)
        #theta3 = math.atan2(sin_theta3, cos_theta3)
        theta3= math.acos(cos_theta3)
        # Calculate the angle for the second joint
        r = math.sqrt(x**2 + y**2)
        alpha = math.atan2(h, r)
        beta = math.acos((a[1]**2 + c**2 - a[2]**2) / (2 * a[1] * c))
        theta2 = alpha + beta
        cos_theta2 = math.cos(theta2)
        sin_theta2 = math.sin(theta2)

        # Calculate the angle for the first joint
        theta1 = math.atan2(y, x)

		# Convert angles to degrees
        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)
        theta3_deg = -math.degrees(theta3)

        # Print the calculated angles
        print(f"Theta1: {theta1_deg:.2f} degrees")
        print(f"Theta2: {theta2_deg:.2f} degrees")
        print(f"Theta3: {theta3_deg:.2f} degrees")
else:
    print("Target is out of reach")

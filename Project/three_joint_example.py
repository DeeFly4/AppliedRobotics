#!/usr/bin/python3

# Using Dynamixel library from https://gitlab.control.lth.se/anders_blomdell/dynamixel
from dynamixel.model.ax_12a import AX_12A
from dynamixel.model.ax_18a import AX_18A
import dynamixel.channel
import time



if __name__ == '__main__':
    theta = [0,0,0]
    servo_positions = [0,0,0]
    for i in range(3):
        theta[i] = int(input(f"Enter the angle for the first joint {i}: "))
    
    servo_positions[0] = int(512 + theta[0]	* 3.45)
    servo_positions[1] = int(823 - theta[1] * 3.45)
    servo_positions[2] = int(512 - theta[2]	* 3.45)
	
    channel = dynamixel.channel.Channel(speed=1000000, device='/dev/ttyUSB0')
    # Define specific joint servos
    servos = [ AX_18A(channel, 1),
               AX_18A(channel, 2),
               AX_12A(channel, 3) ]
    # Set parameters for angle limit and compliance margin
    for s in servos:
        s.torque_enable.write(0)
        print(s.model_number.read(), s.id.read())
        time.sleep(0.1)
        s.cw_angle_limit.write(0)
        s.ccw_angle_limit.write(1023)
        s.cw_compliance_margin.write(1)
        s.ccw_compliance_margin.write(1)
        s.torque_enable.write(1)
        pass
    """
    print([ s.present_position.read() for s in servos ])
    # Joint 1 (set moving speed and go to desired positions)
    servos[0].moving_speed.write(50)
    servos[0].goal_position.write(212)
    time.sleep(3.0)
    print([ s.present_position.read() for s in servos ])
    servos[0].goal_position.write(412)
    time.sleep(3.0)
    print([ s.present_position.read() for s in servos ])
    # Joint 2 (set moving speed and go to desired positions)
    servos[1].moving_speed.write(50)
    servos[1].goal_position.write(112)
    time.sleep(3.0)
    print([ s.present_position.read() for s in servos ])
    servos[1].goal_position.write(412)
    time.sleep(3.0)
    print([ s.present_position.read() for s in servos ])
    # Joint 3 (set moving speed and go to desired positions)
    """
    servos[0].moving_speed.write(100)
    servos[0].goal_position.write(servo_positions[0])
    time.sleep(3.0)
    
    servos[1].moving_speed.write(100)
    servos[1].goal_position.write(servo_positions[1])
    time.sleep(3.0)
    
    servos[2].moving_speed.write(100)
    servos[2].goal_position.write(servo_positions[2])
    time.sleep(3.0)
    """
    print([ s.present_position.read() for s in servos ])
    servos[2].goal_position.write(512+100)
    time.sleep(3.0)
    print([ s.present_position.read() for s in servos ])
    servos[2].goal_position.write(512+300)
    time.sleep(3.0)
    print([ s.present_position.read() for s in servos ])
    servos[2].goal_position.write(512)
    time.sleep(3.0)
    print([ s.present_position.read() for s in servos ])
    for s in servos:
        s.torque_enable.write(0)
    """

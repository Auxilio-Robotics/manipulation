#!/usr/bin/env python3

import time
import stretch_body.robot
import numpy as np
import stretch_body.hello_utils as hu

def pick(robot : stretch_body.robot.Robot, y,z,theta):
    # lines of codes 
    
    # if end of arm at the same z as the object of interest
    # do nothing
    # else
    # move z up in a loop until it matches the desired state
    
    # error correction
    
    # if(robot.lift.status['pos']<z):
    #     robot.lift.move_by(0.05) #arm up or down, 0.1853600414947183 state at stow
    #     robot.push_command()
    #     time.sleep(5)
    
    robot.lift.move_to(z) #arm up or down, 0.1853600414947183 state at stow
    robot.end_of_arm.move_to("wrist_yaw", theta*(np.pi/180))   # wrist right or left; 0 is front # Input degrees into go, it'll go anti clock-wise in that direction
    robot.end_of_arm.move_to("stretch_gripper", 50)       
    robot.push_command()
    time.sleep(7)
    # robot.lift.wait_until_at_setpoint()
    # robot.arm.wait_until_at_setpoint()
    
    robot.arm.move_to(y) #arm extend or retract     #reaches out for the bottle
    robot.arm.wait_until_at_setpoint()
    robot.push_command()
    time.sleep(5)
    

    robot.end_of_arm.move_to("stretch_gripper", -30)       #grabs the bottle 
    robot.push_command()  
    time.sleep(5)
    
    robot.lift.move_to(z + 0.10) #object grabbed, pick it up
    robot.push_command()
    time.sleep(3)
    
    
    #object grabbed and lifted, now pull it back
    
    robot.arm.move_to(0.2) #arm extend or retract     #reaches out for the bottle
    robot.push_command()
    time.sleep(2.5)
    robot.arm.move_to(0) #arm extend or retract     #reaches out for the bottle
    robot.end_of_arm.move_to("wrist_yaw", 180*(np.pi/180))   # wrist right or left; 0 is front # Input degrees into go, it'll go anti clock-wise in that direction
    robot.push_command()
    # robot.arm.wait_until_at_setpoint()
    time.sleep(4)


    
    robot.lift.move_to(0.3) #arm up or down, 0.1853600414947183 state at stow
    robot.push_command()
    # robot.lift.wait_until_at_setpoint()
    time.sleep(7)

    
def place(y,z,theta):
    # lines of codes 
    
    # if end of arm at the same z as the object of interest
    # do nothing
    # else
    # move z up in a loop until it matches the desired state
    
    # error correction
    
    # if(robot.lift.status['pos']<z):
    #     robot.lift.move_by(0.05) #arm up or down, 0.1853600414947183 state at stow
    #     robot.push_command()
    #     time.sleep(5)
    
    robot.lift.move_to(z+0.10) #arm up or down, 0.1853600414947183 state at stow
    robot.push_command()
    time.sleep(7)
    
    robot.end_of_arm.move_to("wrist_yaw", theta*(np.pi/180))   # wrist right or left; 0 is front # Input degrees into go, it'll go anti clock-wise in that direction
    robot.push_command()
    time.sleep(3)
    
    robot.arm.move_to(y) #arm extend or retract     #reaches out for the bottle
    robot.push_command()
    time.sleep(7)
    
    robot.lift.move_to(z) #object grabbed, pick it up
    robot.push_command()
    time.sleep(5)

    robot.end_of_arm.move_to("stretch_gripper", 50)       #grabs the bottle 
    robot.push_command()  
    time.sleep(4)
    
    robot.arm.move_to(0) #arm extend or retract     #reaches out for the bottle
    robot.push_command()
    time.sleep(7)
    
def stow(robot):
    if not robot.pimu.status['runstop_event']:
        robot.stow()
    else:
        robot.logger.warning('Cannot stow while run-stopped')
    
    
if __name__=='__main__':
    
    # Always start with a stowed position
    # Metric : Meters, Degrees
    # The values below will be retrieved by subscribing to the Manipulation - Perception node
    x=0 # If x!=0 , then robot needs to be translated to be in line with the object of interest
    y=0.5 # Arm extension
    z=0.90 # Arm going up # Absolute value of the robot state wrt base # Maxes out at 1.10
    theta=0 # Yaw, 0 for pointing towards the object
    # gripper fully open --> +50, fully closed --> -100      
    robot = stretch_body.robot.Robot()
    robot.startup()
    hu.print_stretch_re_use()
    stow(robot)
    
    pick(robot, y,z,theta) #to go to the object, pick it and lift the object
    # stow(robot)
    robot.stop()
   
      
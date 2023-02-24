#!/usr/bin/env python3

from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import hello_helpers.hello_misc as hm
from std_msgs.msg import String
import json
import numpy as np
from sensor_msgs.msg import JointState
import stretch_body.robot

class TrackObject:

    def __init__(self):

        self.rate = 10.0
        
        self.kp = [10, 10]
        self.kd = [1, 1]
        self.prevxerr = 0
        
    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def getBoundingBox(self, msg):
        # msg = {
        #     'boxes' : boxes.cpu().numpy(),
        #     'box_classes' : classes.cpu().numpy(),
        # }

        msg = json.loads(msg.data)
        loc = np.where(np.array(msg['box_classes']).astype(np.uint8) == 39)[0]
        if len(loc) > 0:
            loc = loc[0]
            self.box = msg['boxes'][loc]
            x1, y1, x2, y2 =  self.box
            x = (x1 + x2)/2
            y = (y1 + y2)/2
            # 720, 1280

            xerr = (x - 120)
            dxerr = (xerr - self.prevxerr)
            command = {
                'joint': 'wrist_extension', 
                'delta': xerr * self.kp[0] + dxerr * self.kd[0] 
            }
            
            # command = {'joint': 'joint_lift', 'delta': self.get_deltas()['translate']}
            self.prevxerr = xerr
            if xerr > 5:
                translate = -0.1
            elif xerr < -5:
                translate = 0.1
            else:
                translate = 0
            
            y_des = 1 - (y)/420. + 0.6
            self.robot.base.translate_by(translate)
            self.robot.lift.move_to(y_des)
            rospy.loginfo(f"{x}, {y} | {self.box} | {xerr} | {y_des}")
            self.robot.base.translate_by(translate)
            self.robot.push_command()


    def main(self):
        
        self.robot = stretch_body.robot.Robot()
        self.robot.startup()
        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        rospy.init_node('control_robot', anonymous=False)
        self.rgb_image_subscriber = rospy.Subscriber("/object_bounding_boxes", String, self.getBoundingBox)

        rospy.loginfo("Node initialized")
    # def send_command(self, command):
    #     joint_state = self.joint_state
    #     if (joint_state is not None) and (command is not None):
    #         point = JointTrajectoryPoint()
    #         point.time_from_start = rospy.Duration(0.0)
    #         trajectory_goal = FollowJointTrajectoryGoal()
    #         trajectory_goal.goal_time_tolerance = rospy.Time(1.0)
    #         joint_name = command['joint']
    #         trajectory_goal.trajectory.joint_names = [joint_name]
    #         if 'inc' in command:
    #             inc = command['inc']
    #             new_value = inc
    #         elif 'delta' in command:
    #             joint_index = joint_state.name.index(joint_name)
    #             joint_value = joint_state.position[joint_index]
    #             delta = command['delta']
    #             new_value = joint_value + delta
    #         point.positions = [new_value]
    #         trajectory_goal.trajectory.points = [point]
    #         trajectory_goal.trajectory.header.stamp = rospy.Time.now()
    #         self.trajectory_client.send_goal(trajectory_goal)

if __name__ == "__main__":
    
    node = TrackObject()
    node.main()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
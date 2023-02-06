#!/usr/bin/env python3
import time
import threading
import argparse as ap        
import numpy as np
import os
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PointStamped, Twist
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from sensor_msgs.msg import PointCloud2, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import hello_helpers.hello_misc as hm

class ManipulationNode(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.letter_height_m = 0.2
        self.wrist_position = None
        self.lift_position = None
        self.manipulation_view = None
        self.debug_directory = None
        self.defined = False
        
    def joint_states_callback(self, joint_states):
        with self.joint_states_lock: 
            self.joint_states = joint_states
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(joint_states)
        self.wrist_position = wrist_position
        lift_position, lift_velocity, lift_effort = hm.get_lift_state(joint_states)
        self.lift_position = lift_position
        self.left_finger_position, temp1, temp2 = hm.get_left_finger_state(joint_states)
        
    def lower_tool_until_contact(self):
        rospy.loginfo('lower_tool_until_contact')
        trigger_request = TriggerRequest() 
        trigger_result = self.trigger_lower_until_contact_service(trigger_request)
        rospy.loginfo('trigger_result = {0}'.format(trigger_result))
        
    def trigger_picking_pipeline_callback(self, request):
        try:
            rospy.loginfo("pick service called")
            pose = {'joint_head_tilt':(np.deg2rad(-90))/3}
            self.move_to_pose(pose)
            # TODO
            # Read data from camera, and crop the region of interest. Get X,Y,Z coordinates.
            # Initial naive method will only do frontal grasp. 
            # close the loop using the x, y coordinate by moving such that the grasping location is
            # exactly in line with the gripper. 
            # This can be done without using depth information.
            # Get new object detections. get xyz from camera and ROI cropping
            # using the z obtained, just go for the grasp :)
            #  in this method we dont care about planes.
            if self.goal_x and self.goal_y and self.goal_z:
                poses = [
                    # Step 0 - Home position
                    {'joint_lift': 0.82,'wrist_extension': 0.0, 'joint_gripper_finger_left':open},
                    # Step 1 - Rotate to bring the cup in center
                    {'rotate_mobile_base': np.arctan(self.goal_y/np.abs(self.goal_x))},
                    # Step 2 - Rotate Head
                    {'joint_head_pan':0},
                    # Step 3 - Translate
                    {'translate_mobile_base': self.goal_x - 0.6},
                    # Step 4
                    {'rotate_mobile_base': 1.57},
                    # Step 5
                    {'joint_head_pan':-1.57},
                    # Step 6
                    {'translate_mobile_base': self.goal_x},
                    # Step 7
                    {'translate_mobile_base': offset[0]},
                    # Step 8
                    {'joint_lift': self.goal_z},
                    # Step 9
                    {'wrist_extension': (-(self.goal_y)) - 0.36},
                    # Step 10
                    {'joint_gripper_finger_left':close},
                    # Step 11
                    {'joint_lift':1.05},
                    # Step 12
                    {'wrist_extension':0},
                    # Step 13
                    {'joint_head_tilt':(np.deg2rad(-90))/3, 'joint_head_pan':0}
                ]
            # 3,7,8,10
            for i in range(len(poses)):
                rospy.loginfo("===================================================================")
                rospy.loginfo("Performing step : "+str(i)+" : "+str(poses[i]))
                if (i==0):
                    time_sleep = 1
                if (i==1):
                    time_sleep = 0.02 * (np.rad2deg(np.arctan(self.goal_y/np.abs(self.goal_x))))
                if(i==2):
                    time_sleep = 1 # Changed from 2 to 1
                if(i==3):
                    error = 1000
                    while(np.abs(error) > 0.005):
                        resp1 = perception_picking_client(object_id, 1)
                        if(resp1.detected==False):
                            for i in range(5):
                                resp1 = perception_picking_client(object_id, 1)
                                if(resp1.detected==True):
                                    break
                            else:
                                print("Not detected")
                                return
                        self.goal_x = resp1.x_reply
                        self.goal_y = resp1.y_reply
                        self.goal_z = resp1.z_reply
                        error = self.goal_x - 0.6
                        rospy.loginfo("Correcting error : "+str(error))
                        poses[i] = {'translate_mobile_base':error}
                        self.move_to_pose(poses[i])
                        time.sleep(3*(np.abs(error)))

                        resp1 = perception_picking_client(object_id, 1)
                        if(resp1.detected==False):
                            for i in range(5):
                                resp1 = perception_picking_client(object_id, 1)
                                if(resp1.detected==True):
                                    break
                            else:
                                print("Not detected")
                                return
                        self.goal_x = resp1.x_reply
                        self.goal_y = resp1.y_reply
                        self.goal_z = resp1.z_reply
                        error = self.goal_x - 0.6
                    rospy.loginfo("Error after step 3 : "+str(error))
                    continue
                    
                if (i==4 or i==5):
                    time_sleep = 2
                if(i==6):
                    error = 1000
                    while(np.abs(error) > 0.008):
                        resp1 = perception_picking_client(object_id, 2)
                        if(resp1.detected==False):
                            for i in range(5):
                                resp1 = perception_picking_client(object_id, 2)
                                if(resp1.detected==True):
                                    break
                            else:
                                print("Not detected")
                                return
                        self.goal_x = resp1.x_reply
                        self.goal_y = resp1.y_reply
                        self.goal_z = resp1.z_reply
                        error = self.goal_x
                        rospy.loginfo("Correcting error : "+str(error))
                        poses[i] = {'translate_mobile_base':error}
                        self.move_to_pose(poses[i])
                        time.sleep(5*(np.abs(error)))

                        resp1 = perception_picking_client(object_id, 2)
                        if(resp1.detected==False):
                            for i in range(5):
                                resp1 = perception_picking_client(object_id, 2)
                                if(resp1.detected==True):
                                    break
                            else:
                                print("Not detected")
                                return
                        self.goal_x = resp1.x_reply
                        self.goal_y = resp1.y_reply
                        self.goal_z = resp1.z_reply
                        error = self.goal_x #+ 0.03
                    rospy.loginfo("Error after step 7 : "+str(error))
                    continue
                if (i==7):
                    time.sleep(2)
                if (i==8):
                    resp1 = perception_picking_client(object_id, 2)
                    if(resp1.detected==False):
                        for i in range(5):
                            resp1 = perception_picking_client(object_id, 2)
                            if(resp1.detected==True):
                                break
                        else:
                            print("Not detected")
                            return
                    self.goal_x = resp1.x_reply
                    self.goal_y = resp1.y_reply
                    self.goal_z = resp1.z_reply
                    poses[i] = {'joint_lift': self.goal_z + offset[2]}
                    time_sleep = 2
                if(i==9):
                    rospy.loginfo("self.goal_y"+str(self.goal_y))
                    extension = (-(self.goal_y))-0.26 + offset[1]
                    rospy.loginfo("extension : "+str(extension))
                    # while(self.goal_y > 0.01):
                        # print("extension",extension)
                    if extension > 0.5:
                        extension = 0.5
                    poses[i] = {'wrist_extension':extension}
                    self.move_to_pose(poses[i])
                    time.sleep(10*(extension))
                    self.move_to_pose(poses[i])
                    # extension = (-(self.goal_y))-0.36
                    continue
                if(i==10 or i==11):
                    time_sleep = 2
                self.move_to_pose(poses[i])

                sl = np.abs((-(self.goal_y))-0.26) * 10
                time.sleep(np.abs(time_sleep))
                rospy.loginfo("Completed performing step : "+str(i))

                if(i==12 or i==13):
                    time_sleep = 2

            # # Picking completed
            rospy.loginfo("Picking completed")
            if(self.trigger_drop_object_callback()==True):
                rospy.loginfo("Dropping completed")
                return grasp_serviceResponse(True)
            else:
                return grasp_serviceResponse(False)
        except Exception as e:
            print("Error : ",e)
            fail_message =  "Could not run picking pipeline"
            # sendFinishMessage(fail_message)
        
    
    def main(self):
        
        hm.HelloNode.main(self, 'grasp_object', 'grasp_object', wait_for_first_pointcloud=False)
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        self.trigger_grasp_object_service = rospy.Service('/grasp_object/grasp_service_call',
                                                           grasp_service,
                                                           self.trigger_picking_pipeline_callback)
        rospy.loginfo("After service - Service server started")
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()
        
# -----------------------------------------------------------------------------
        
if __name__ == '__main__':
    try:
        parser = ap.ArgumentParser(description='Manipulation behavior for stretch.')
        args, unknown = parser.parse_known_args()
        node = ManipulationNode()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')   



    
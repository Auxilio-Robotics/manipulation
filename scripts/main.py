#!/usr/bin/env python
#!/usr/bin/env python3
"""
Naive manipulation code
"""

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
        
    def move_to_initial_configuration(self):
        initial_pose = {'wrist_extension': 0.01,
                        'joint_wrist_yaw': 0.0,
                        'gripper_aperture': 0.125}

        rospy.loginfo('Move to the initial configuration for drawer opening.')
        # self.move_to_pose(initial_pose)

    def drive(self, forward_m):
        tolerance_distance_m = 0.005
        if forward_m > 0: 
            at_goal = self.move_base.forward(forward_m, detect_obstacles=False, tolerance_distance_m=tolerance_distance_m)
        else:
            at_goal = self.move_base.backward(forward_m, detect_obstacles=False, tolerance_distance_m=tolerance_distance_m)

    def trigger_picking_pipeline_callback(self, request):
        try:

            rospy.loginfo("pick service called")
            pose = {'joint_head_tilt':(np.deg2rad(-90))/3}
            self.move_to_pose(pose)

            object_id = request.object_id
            offsets = np.zeros((12,3))

            offsets[object_id][0] += 0.07 # for the left(+)/right(-) positioning of gripper
            offsets[object_id][1] -= 0.05 # for the extension
            offsets[object_id][2] -= 0.03 # for the height

            # id=0,  => 'cmu_tartan_bottle'
            offsets[0][0] += 0.01

            # # id=1,  => 'tennis_ball_toy'
            offsets[1][1] -= 0.02
            offsets[1][2] -= 0.01
            # # id=2,  => 'cmu_cup'
            # offsets[2][0]+= 0.07
            # offsets[2][2]+=0.03
            # # id=3,  => 'cmu_bottle'
            # offsets[3][0]+=0.03
            # offsets[3][1]+=0.01
            # offsets[3][2]+=0.02

            # # id=4,  => 'monkey_keychain'
            # offsets[4][0] +=0.03
            # offsets[4][2] += 0.07
            # # id=5,  => 'transparent_bottle'
            # ##REJECTED!!!!!!!!!!!

            # # id=6,  => 'all_star_dogs_belt'
            offsets[6][1] -= 0.01
            # offsets[6][2] = 0.08
            # # id=7,  => 'dog_collar'
            # offsets[7][0] = 0.03
            # offsets[7][2] = 0.03
            # # id=8,  => 'cow_keychain'
            offsets[8][1] -= 0.015
            # offsets[8][2] += 0.07
            # # id=9,  => 'beanie'
            # offsets[9][0] = 0.07
            # offsets[9][2] = 0.05
            # # id=10,  => 'unicorn'
            # offsets[10][0] += 0.07

            # offsets[10][2] += 0.03
            # id=11,  => 'airpods_case'
            # offsets[11][0] += 0.015
            offsets[11][0] += 0.02
            offset = offsets[object_id]
            
            # Request centroid from picking service
            rospy.wait_for_service('perception_picking_service')
            try:
                perception_picking_client = rospy.ServiceProxy('perception_picking_service', perception_picking)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return

            # Step 0 : Moving the camera to detect object 
            i = 1
            time.sleep(5)
            flag = False
            while flag==False:
                resp1 = perception_picking_client(object_id, 1)
                flag = resp1.detected
                if flag:
                    self.goal_x = resp1.x_reply
                    self.goal_y = resp1.y_reply
                    self.goal_z = resp1.z_reply
                    break
                
            open = 0.15283721447
            close=-0.37
                
            # Steps
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
        
    def trigger_drop_object_callback(self):
        rospy.loginfo("grasp service called")
        pose = {'joint_head_tilt':(np.deg2rad(-90))/3}
        self.move_to_pose(pose)
        rospy.wait_for_service('perception_shipping_service')
        try:
            perception_drop_client = rospy.ServiceProxy('perception_shipping_service', perception_shipping)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return

        # Tilt angle for the robot : -0.6149752152689414
        # Second angle - -0.702412120178423
        # Step 0 : Moving the camera to detect object 
        i = 1
        time.sleep(2)
        flag = False
        while flag==False:
            resp1 = perception_drop_client(True)
            flag = resp1.detected
            if flag:
                print("Detected dropping box")
                self.goal_x = resp1.x_reply
                self.goal_y = resp1.y_reply
                self.goal_z = resp1.z_reply
                break
            
            rospy.loginfo("Value:"+str(flag))
            rospy.loginfo("Rotating clockwise : "+str(i*20)+"by 20 degrees")
            # rot_angle = np.deg2rad(-20) * i
            pose = {'joint_head_pan': np.deg2rad(20) * i}
            i=i+1
            if i>9:
                # print("i : ",i)
                rospy.loginfo("Rotating clockwise : "+str((i-9)*20)+"by 10 degrees")
                pose = {'joint_head_pan': np.deg2rad(-20) * (i-9)}
            rospy.loginfo(pose)
            
            if(i>13):
                # print("i : ",i)
                rospy.loginfo("Couldn't detect object")
                pose = {'joint_head_pan': 0}
                self.move_to_pose(pose)
                return
            self.move_to_pose(pose)
            rospy.loginfo("Done moving")
            time.sleep(2)

        print("Outside while loop")
        open  = 0.15283721447
        close =-0.37
            
        # Steps
        if self.goal_x and self.goal_y and self.goal_z:
            poses = [
                # Step 0 - Home position
                {'joint_lift': 0.9,'wrist_extension': 0.0, 'joint_gripper_finger_left':close},
                # Step 1 - Rotate to bring the cup in center
                {'rotate_mobile_base': np.arctan(self.goal_y/np.abs(self.goal_x))},
                # Step 2 - Rotate Head
                {'joint_head_pan':0},
                # Step 3 - Translate
                {'translate_mobile_base': self.goal_x - 0.7},
                # Step 4
                # {'joint_lift': self.goal_z + 0.03},
                # {'joint_lift': 0.9},
                # Step 4
                {'rotate_mobile_base': 1.57},
                # Step 5
                {'joint_lift': 0.3,'joint_head_pan':-1.57},
                # Step 6
                {'translate_mobile_base': self.goal_x},
                # Step 7
                {'translate_mobile_base': 0.1},
                # Step 8
                {'joint_lift':0.9},
                # Step 9
                {'wrist_extension': (-(self.goal_y)) - 0.36},
                # Step 10
                {'joint_gripper_finger_left':open},
                # Step 11
                {'joint_lift':0.9},
                # Step 12
                {'wrist_extension':0},
                # Step 13
                {'joint_head_tilt':((np.deg2rad(-90))/3), 'joint_head_pan':0},
                # Step 14
                {'rotate_mobile_base': -3.14}
            ]
        # 3,7,8,10
        for i in range(len(poses)):
            rospy.loginfo("===================================================================")
            rospy.loginfo("Performing step : "+str(i)+" : "+str(poses[i]))
            if (i==0):
                time_sleep = 1
            if (i==1):
                time_sleep = 0.03 * (np.rad2deg(np.arctan(self.goal_y/self.goal_x)))
            if(i==2):
                time_sleep = 1
            if(i==3):
                error = 1000
                while(np.abs(error) > 0.05):
                    resp1 = perception_drop_client(True)
                    if(resp1.detected==False):
                        for i in range(8):
                            resp1 = perception_drop_client(True)
                            if(resp1.detected==True):
                                break
                        else:
                            print("Not detected")
                            return
                    self.goal_x = resp1.x_reply
                    self.goal_y = resp1.y_reply
                    self.goal_z = resp1.z_reply + 0.05
                    error = self.goal_x - 0.7
                    rospy.loginfo("Correcting error : "+str(error))
                    poses[i] = {'translate_mobile_base':error}
                    self.move_to_pose(poses[i])
                    time.sleep(5*(np.abs(error)))

                    resp1 = perception_drop_client(True)
                    print("Checking detections again")
                    if(resp1.detected==False):
                        for i in range(5):
                            print("Iteration : ",i)
                            resp1 = perception_drop_client(True)
                            if(resp1.detected==True):
                                break
                        else:
                            print("Not detected")
                            return
                    print("Inside while loop.")
                    self.goal_x = resp1.x_reply
                    self.goal_y = resp1.y_reply
                    self.goal_z = resp1.z_reply + 0.05
                    error = self.goal_x - 0.7
                rospy.loginfo("Error after step 3 : "+str(error))
                continue

               
            # if(i==4):
            #     time_sleep = 1
            if (i==4 or i==5):
                time_sleep = 2
            if(i==6):
                error = 1000
                while(np.abs(error) > 0.05):
                    resp1 = perception_drop_client(True)
                    if(resp1.detected==False):
                        for i in range(5):
                            resp1 = perception_drop_client(True)
                            if(resp1.detected==True):
                                break
                        else:
                            print("Not detected")
                            return
                    self.goal_x = resp1.x_reply
                    self.goal_y = resp1.y_reply
                    self.goal_z = resp1.z_reply + 0.05
                    error = self.goal_x #+ 0.03
                    rospy.loginfo("Correcting error : "+str(error))
                    poses[i] = {'translate_mobile_base':error}
                    self.move_to_pose(poses[i])
                    time.sleep(3*(np.abs(error)))

                    resp1 = perception_drop_client(True)
                    if(resp1.detected==False):
                        for i in range(5):
                            resp1 = perception_drop_client(True)
                            if(resp1.detected==True):
                                break
                        else:
                            print("Not detected")
                            return
                    self.goal_x = resp1.x_reply
                    self.goal_y = resp1.y_reply
                    self.goal_z = resp1.z_reply 
                    error = self.goal_x 
                rospy.loginfo("Error after step 7 : "+str(error))
                # error = self.goal_x #+ 0.03
                # rospy.loginfo("error : "+str(error))
                # while(np.abs(error) > 0.006):
                #     # if error>0:
                    
                #     rospy.loginfo("Correcting error : "+str(error))
                #     poses[i] = {'translate_mobile_base': error}
                #     # self.move_to_pose(poses[i])
                #     time.sleep(5*(np.abs(error)))
                #     error = self.goal_x #+ 0.03
                # rospy.loginfo("Error after step 7 : "+str(error))
                continue
            
            if(i==7 or i==8):
                time_sleep = 2
            if(i==9):
                rospy.loginfo("self.goal_y"+str(self.goal_y))
                extension = (-(self.goal_y))-0.26 - 0.12
                rospy.loginfo("extension : "+str(extension))
                # while(self.goal_y > 0.01):
                    # print("extension",extension)
                if extension > 0.5:
                    extension = 0.5
                poses[i] = {'wrist_extension':extension}
                
                self.move_to_pose(poses[i])
                time.sleep(10*(extension))
                # # self.move_to_pose(poses[i])
                    # extension = (-(self.goal_y))-0.36
                continue
            if(i==10 or i==11):
                time_sleep = 2
            self.move_to_pose(poses[i])
            # rospy.loginfo("Completed calling : "+str(i))
            sl = np.abs((-(self.goal_y))-0.26) * 10
            # rospy.loginfo("Sleeping for : "+str(time_sleep))
            time.sleep(np.abs(time_sleep))
            rospy.loginfo("Completed performing step : "+str(i))

            if(i==13 or i==14 or i==15):
                time_sleep = 2


        rospy.loginfo('Completed object drop!')
        return True
        # return TriggerResponse(
        #     success=True,
        #     message='Completed object drop!'
        #     )

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



    
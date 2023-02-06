from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import hello_helpers.hello_misc as hm
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import String
import json
import base64
import numpy as np

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


def json_numpy_obj_hook(dct):
    """
    Decodes a previously encoded numpy ndarray
    with proper shape and dtype
    :param dct: (dict) json encoded ndarray
    :return: (ndarray) if input was an encoded ndarray
    """
    if isinstance(dct, dict) and '__ndarray__' in dct:
        data = base64.b64decode(dct['__ndarray__'])
        return np.frombuffer(data, dct['dtype']).reshape(dct['shape'])
    return dct

# Overload dump/load to default use this behavior.
def dumps(*args, **kwargs):
    kwargs.setdefault('cls', NumpyEncoder)
    return json.dumps(*args, **kwargs)

def loads(*args, **kwargs):
    kwargs.setdefault('object_hook', json_numpy_obj_hook)    
    return json.loads(*args, **kwargs)  

def dump(*args, **kwargs):
    kwargs.setdefault('cls', NumpyEncoder)
    return json.dump(*args, **kwargs)

def load(*args, **kwargs):
    kwargs.setdefault('object_hook', json_numpy_obj_hook)
    return json.load(*args, **kwargs)

class TrackObject(hm.HelloNode):

    def __init__(self):
        super().__init__()

        self.rate = 10.0
        self.rgb_image_subscriber = rospy.Subscriber("chatter", String, self.getBoundingBox)
        self.kp = [10, 10]
        self.kd = [1, 1]
        self.prevxerr = 0
    def getBoundingBox(self, msg):
        # msg = {
        #     'boxes' : boxes.cpu().numpy(),
        #     'box_classes' : classes.cpu().numpy(),
        # }
        msg = loads(msg)
        loc = np.where(msg['box_classes'].astype(np.uint8) == 39)
        self.box = msg['boxes'][loc]
        x1, y1, x2, y2 =  self.box
        x = (x1 + x2)/2
        y = (y1 + y2)/2
        # 720, 1280

        xerr = (x - 360)/10.0
        dxerr = (xerr - self.prevxerr)
        command = {
            'joint': 'translate_mobile_base', 
            'inc': xerr * self.kp[0] + dxerr * self.kd[0] 
        }
        self.send_command(command)
        self.prevxerr = xerr

    def main(self):
        rospy.init_node('control_robot', anonymous=False)
        rospy.loginfo("Node initialized")
        rospy.spin()

    def send_command(self, command):
        joint_state = self.joint_state
        if (joint_state is not None) and (command is not None):
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.0)
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(1.0)
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]
            if 'inc' in command:
                inc = command['inc']
                new_value = inc
            elif 'delta' in command:
                joint_index = joint_state.name.index(joint_name)
                joint_value = joint_state.position[joint_index]
                delta = command['delta']
                new_value = joint_value + delta
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            self.trajectory_client.send_goal(trajectory_goal)
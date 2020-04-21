#!/usr/bin/env python
import rospy
from champ_msgs.msg import Pose
import numpy as np
import math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

class PoseCmd:
    def __init__(self):
        self.pose_subscriber = rospy.Subscriber('champ/cmd_pose', Pose, self.pose_callback)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def pose_callback(self, data):
        self.roll = data.roll
        self.pitch = data.pitch
        self.yaw = data.yaw

def rotate(pos, alpha, phi, beta):
    rx = np.array(((1,             0, 0), 
                   (0, np.cos(alpha), -np.sin(alpha)),
                   (0, np.sin(alpha),  np.cos(alpha)) ))
    new_pos = rx.dot(pos)

    ry = np.array((( np.cos(phi), 0, np.sin(phi)), 
                   (           0, 1, 0),
                   (-np.sin(phi), 0, np.cos(phi)) ))
    new_pos = ry.dot(new_pos)


    rz = np.array(((np.cos(beta), -np.sin(beta), 0), 
                   (np.sin(beta),  np.cos(beta), 0),
                   (           0,             0, 1) ))
    new_pos = rz.dot(new_pos)

    return new_pos

if __name__ == "__main__":
    rospy.init_node("robot_api_test", anonymous = True)
    joint_states_pub = rospy.Publisher('/champ/arm/joint_states', JointState, queue_size = 100)

    joint_names = ["base_joint", "lower_arm_joint", "upper_arm_joint", "wrist1_joint", "wrist2_joint"]

    pose_cmd = PoseCmd()

    target_pos = np.array((0.391, 0.0, 0.165))
    base_to_lower_arm = np.array((0.070, 0.000, 0.100))
    lower_to_upper_arm = np.array((0.195, 0.000, 0.000))
    upper_arm_to_wrist1 = np.array((0.195, 0.000, 0.000))
    wrist1_to_wrist2 = np.array((0.065, 0.000, 0.000))

    transformed_target = target_pos - base_to_lower_arm

    while not rospy.is_shutdown():
        
        new_pos = rotate(target_pos, -pose_cmd.roll, -pose_cmd.pitch, -pose_cmd.yaw)
        new_pos = new_pos - base_to_lower_arm
        new_pos = new_pos - wrist1_to_wrist2

        theta = math.atan(new_pos[1] / new_pos[0])

        l1 = lower_to_upper_arm[0]
        l2 = upper_arm_to_wrist1[0]
        x = new_pos[0]
        y = new_pos[2]

        upper_joint = math.acos((x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2))
        lower_joint = -math.atan(y / x) - math.atan((l2 * math.sin(upper_joint)) / (l1 + (l2 * math.cos(upper_joint))))
   
        alpha = math.pi - upper_joint
        beta = math.pi - abs(alpha) - abs(lower_joint)

        joint_states = JointState()
        joint_states.header.stamp = rospy.Time.now()
        joint_states.name = joint_names

        joint_states.position.append(theta)
        joint_states.position.append(lower_joint)
        joint_states.position.append(upper_joint)
        joint_states.position.append(-beta - pose_cmd.pitch)
        joint_states.position.append(-pose_cmd.roll)

        joint_states_pub.publish(joint_states)

        rospy.sleep(0.01)
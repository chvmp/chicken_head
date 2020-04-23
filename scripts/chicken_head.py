#!/usr/bin/env python
import rospy
from champ_msgs.msg import Pose
import numpy as np
import math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion

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
    rz = np.array(((np.cos(beta), -np.sin(beta), 0), 
                   (np.sin(beta),  np.cos(beta), 0),
                   (           0,             0, 1) ))
    target_pos = rz.dot(pos)

    ry = np.array((( np.cos(phi), 0, np.sin(phi)), 
                   (           0, 1, 0),
                   (-np.sin(phi), 0, np.cos(phi)) ))

    target_pos = ry.dot(target_pos)

    rx = np.array(((1,             0, 0), 
                   (0, np.cos(alpha), -np.sin(alpha)),
                   (0, np.sin(alpha),  np.cos(alpha)) ))
    target_pos = rx.dot(target_pos)

    return target_pos


if __name__ == "__main__":
    rospy.init_node("robot_api_test", anonymous = True)
    joint_states_pub = rospy.Publisher('/champ/arm/joint_states', JointState, queue_size = 100)

    joint_names = ["base_joint", "lower_arm_joint", "upper_arm_joint", "wrist1_joint", "wrist2_joint"]

    pose_cmd = PoseCmd()

    initial_pos = np.array((0.391, 0.0, 0.165))
    base_to_lower_arm = np.array((0.070, 0.000, 0.100))
    lower_to_upper_arm = np.array((0.195, 0.000, 0.000))
    upper_arm_to_wrist1 = np.array((0.195, 0.000, 0.000))
    wrist1_to_wrist2 = np.array((0.065, 0.000, 0.000))

    while not rospy.is_shutdown():
        target_pos = rotate(initial_pos, -pose_cmd.roll, -pose_cmd.pitch, 0)
        target_pos = target_pos - base_to_lower_arm
  
        temp_pos = rotate(initial_pos, -pose_cmd.roll, -pose_cmd.pitch, -pose_cmd.yaw)
        temp_pos = temp_pos - wrist1_to_wrist2
        theta = math.atan2(temp_pos[1],temp_pos[0])

        l1 = lower_to_upper_arm[0]
        l2 = upper_arm_to_wrist1[0]
        x = target_pos[0]
        y = target_pos[2]

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
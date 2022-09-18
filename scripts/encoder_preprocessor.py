# !/usr/bin/python

import rosbag, rospy
from nav_msgs.msg import Odometry
from kaist_tools.msg import EncoderTick
import tf.transformations as tr
from geometry_msgs.msg import TransformStamped

import sys
from math import cos, sin, isnan
import numpy as np

from scipy.spatial.transform import Rotation as R

from time import sleep

#
# Convert encoder tick data from custom ROS message to odometry data
#
# To run the node, it's better to use launch file providing topics as parameters
#

class Converter:
    def __init__(self, input_topic, output_topic):
        rospy.loginfo(
            "Encoder data preprocessing: \n \t input_topic:  " + input_topic + "\n \t output topic: " + output_topic)

        self.pub = rospy.Publisher(output_topic, Odometry, queue_size=100)
        self.sub = rospy.Subscriber(input_topic, EncoderTick, self.encoder_cbk, queue_size=100)

        # ***

        # Urban 29
        # world_global_Rij = np.array([[-0.9986975182, -0.02340548807, -0.04533707308], [0.02381712338, -0.9996796793, -0.008560570362], [-0.04512218635, -0.009629219038, 0.9989350662]])
        # world_global_tij = np.array([[332538.7257, 4136123.762, 17.98436895]]).transpose()

        # Urban 21
        # world_global_Rij = np.array([[0.0127029128, -0.9998056982, 0.01507321485], [0.9996953553, 0.0130176563, 0.02096991422], [-0.02116205765, 0.01480224388, 0.9996664748]])
        # world_global_tij = np.array([[338677.8155, 4086542.602, 18.48196375]]).transpose()

        # Urban 26
        world_global_Rij = np.array(
            [[-0.9997840965, -0.0193252349, 0.007635158537], [0.01942543815, -0.9997231711, 0.01327529918],
             [0.007376496629, 0.0134207493, 0.9998827285]])
        world_global_tij = np.array([[330089.6744, 4118151.702, 18.03757139]]).transpose()

        # ***

        self.world_global_Hij = np.block([world_global_Rij, world_global_tij])
        self.world_global_Hij = np.concatenate((self.world_global_Hij, np.array([[0, 0, 0, 1]])))

        self.resolution = 4096
        self.left_wheel_diameter = 0.623479
        self.right_wheel_diameter = 0.622806
        self.wheel_base = 1.52439

        self.left_distance_per_count = (np.pi * self.left_wheel_diameter) / self.resolution
        self.right_distance_per_count = (np.pi * self.right_wheel_diameter) / self.resolution

        self.left_count_prev = 0.0
        self.right_count_prev = 0.0
        self.timestamp_last = rospy.Time.now()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.first = True

    def encoder_cbk(self, msg):

        timestamp = msg.header.stamp

        delta_left_count = msg.left_tick.data - self.left_count_prev
        delta_right_count = msg.right_tick.data - self.right_count_prev

        if self.first:
            self.left_count_prev = msg.left_tick.data
            self.right_count_prev = msg.right_tick.data
            self.timestamp_last = timestamp
            self.first = False
            return

        dt = timestamp.to_sec() - self.timestamp_last.to_sec()

        d_l = delta_left_count * self.left_distance_per_count
        d_r = delta_right_count * self.right_distance_per_count

        lin_vel = ((d_r + d_l) / 2) / dt
        ang_vel = ((d_r - d_l) / self.wheel_base) / dt

        delta_x = lin_vel * cos(self.theta) * dt
        delta_y = lin_vel * sin(self.theta) * dt
        delta_theta = ang_vel * dt

        # TODO: When to update? Before or after?
        self.theta += delta_theta  # Preintegration from the report makes y positive, which seems to be more incorrect

        if not isnan(delta_x):
            self.x = self.x + delta_x  # TODO: Doubts about the sign btw sins
        if not isnan(delta_y):
            self.y = self.y + delta_y

        quaternion = tr.quaternion_from_euler(0.0, 0.0, self.theta)

        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = "vehicle"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        self.left_count_prev = msg.left_tick.data
        self.right_count_prev = msg.right_tick.data
        self.timestamp_last = timestamp

        self.pub.publish(odom_msg)


if __name__ == '__main__':
    rospy.init_node("encoder_data_preprocessor")
    input_topic = rospy.get_param('~encoder_data_topic')

    output_topic = rospy.get_param('~wheel_odometry_topic')
    converter = Converter(input_topic, output_topic)
    rospy.spin()

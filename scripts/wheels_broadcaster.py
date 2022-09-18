#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, Path

class Broadcaster:
    def __init__(self):
        self.sub = rospy.Subscriber("/odom", Odometry, self.cb)
        self.pub = rospy.Publisher("/path", Path, queue_size=1)

        self.path = Path()

    def cb(self, msg):
        br = tf.TransformBroadcaster()
        br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                         (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                          msg.header.stamp,
                          "odom",
                          "vehicle")

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = msg.header.frame_id
        pose.pose.position.x = msg.pose.pose.position.x
        pose.pose.position.y = msg.pose.pose.position.y
        pose.pose.position.z = msg.pose.pose.position.z
        pose.pose.orientation.x = msg.pose.pose.orientation.x
        pose.pose.orientation.y = msg.pose.pose.orientation.y
        pose.pose.orientation.z = msg.pose.pose.orientation.z
        pose.pose.orientation.w = msg.pose.pose.orientation.w

        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = "map"
        self.path.poses.append(pose)

        self.pub.publish(self.path)

if __name__ == '__main__':
    rospy.init_node("wheels_broadcaster")
    broadcaster = Broadcaster()
    rospy.spin()
# !/usr/bin/python
#
# Convert the wheel encoders csv files to a rosbag
#
# To call:
#
#   python wheels_to_rosbag.py encoder.csv wheels.bag
#

import rosbag, rospy
from nav_msgs.msg import Odometry
import tf.transformations as tr
from geometry_msgs.msg import TransformStamped

import sys
from math import cos, sin, isnan
import numpy as np

from scipy.spatial.transform import Rotation as R

from time import sleep

class Converter:
    def __init__(self, args):
        if len(args) < 2:
            print("Please specify encoders file")
            return 1

        if len(args) < 3:
            print("Please, specify output rosbag file")
            return 1

        self.wheels = np.loadtxt(args[1], delimiter=',')

        self.global_poses = np.empty((self.wheels.shape[0], 8))

        self.file_bag = args[2]
        
        self.bag = rosbag.Bag(self.file_bag, 'w')

        self.resolution = 4096
        self.left_wheel_diameter = 0.623479
        self.right_wheel_diameter = 0.622806
        self.wheel_base = 1.52439
        
        # self.left_distance_per_count = (2 * 3.14159265) / self.resolution
        # self.right_distance_per_count = (2 * 3.14159265) / self.resolution
        
        self.left_distance_per_count = (np.pi * self.left_wheel_diameter) / self.resolution
        self.right_distance_per_count = (np.pi * self.right_wheel_diameter) / self.resolution

        self.left_count_prev = 0.0
        self.right_count_prev = 0.0

        self.timestamp_last = None

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.time_factor = 1e9  # ns -> s
        self.interp_delta_t = 0.005

        self.first = True

    def interp_data(self, x, t, t0):

        x_int = np.zeros((t.shape[0], x.shape[1]))

        for i in range(0, x.shape[1]):
            x_int[:, i] = np.interp(t, (x[:, 0] - t0) / self.time_factor, x[:, i])

        return x_int

    def main(self):

        # Urban 29
        # world_global_Rij = np.array([[-0.9986975182, -0.02340548807, -0.04533707308], [0.02381712338, -0.9996796793, -0.008560570362], [-0.04512218635, -0.009629219038, 0.9989350662]])
        # world_global_tij = np.array([[332538.7257, 4136123.762, 17.98436895]]).transpose()

        # Urban 21
        # world_global_Rij = np.array([[0.0127029128, -0.9998056982, 0.01507321485], [0.9996953553, 0.0130176563, 0.02096991422], [-0.02116205765, 0.01480224388, 0.9996664748]])
        # world_global_tij = np.array([[338677.8155, 4086542.602, 18.48196375]]).transpose()

        # Urban 26
        world_global_Rij = np.array([[-0.9997840965, -0.0193252349, 0.007635158537], [0.01942543815, -0.9997231711, 0.01327529918], [0.007376496629, 0.0134207493, 0.9998827285]])
        world_global_tij = np.array([[330089.6744, 4118151.702, 18.03757139]]).transpose()
        
        world_global_Hij = np.block([world_global_Rij, world_global_tij])
        world_global_Hij = np.concatenate((world_global_Hij, np.array([[0, 0, 0, 1]])))

        t0 = self.wheels[0, 0]
        t_end = self.wheels[-1, 0]

        # Interpolate all
        # Transform differential measurement into integrated measurement
        t_end = int((t_end - t0) / self.time_factor)  # TODO: Suspect int
        t = np.linspace(0, t_end, num=int(t_end / self.interp_delta_t))

        self.wheels = self.interp_data(self.wheels, t, t0)

        times_usec = self.wheels[:, 0]

        left_count = self.wheels[:, 1]
        right_count = self.wheels[:, 2]

        try:

            for i, time_usec in enumerate(times_usec):

                print("Processing timestamp: " + str(time_usec))

                timestamp = rospy.Time.from_sec(time_usec / 1e9)

                delta_left_count = left_count[i] - self.left_count_prev  # TODO: Check it in ML-VIWO
                delta_right_count = right_count[i] - self.right_count_prev

                if self.first:
                    self.left_count_prev = left_count[i]
                    self.right_count_prev = right_count[i]
                    self.timestamp_last = timestamp
                    self.first = False
                    continue

                dt = timestamp.to_sec() - self.timestamp_last.to_sec()  # TODO: Check it in ML-VIWO

                d_l = delta_left_count * self.left_distance_per_count
                d_r = delta_right_count * self.right_distance_per_count

                lin_vel = ((d_r + d_l) / 2) / dt
                ang_vel = ((d_r - d_l) / self.wheel_base) / dt

                # omega_left = delta_left_count * self.left_distance_per_count
                # omega_right = delta_right_count * self.right_distance_per_count

                # omega_left = (delta_left_count * self.left_distance_per_count) / dt
                # omega_right = (delta_right_count * self.right_distance_per_count) / dt

                # v_left = omega_left * self.left_wheel_diameter / 2
                # v_right = omega_right * self.right_wheel_diameter / 2

                # vx = (v_right + v_left) / 2
                # vy = 0.0
                # vtheta = (v_right - v_left) / self.wheel_base

                delta_x = lin_vel * cos(self.theta) * dt
                delta_y = lin_vel * sin(self.theta) * dt
                delta_theta = ang_vel * dt

                # delta_theta = vtheta * dt
                # delta_x = (vx * (sin(self.theta) - sin(self.theta - delta_theta))) / vtheta
                # delta_y = (vx * (cos(self.theta) - cos(self.theta - delta_theta))) / vtheta

                # delta_x = (vx * cos(delta_theta)) * dt
                # delta_y = (vx * sin(delta_theta)) * dt  # Making signs negative doesn't give new results (no impact)
                
                # delta_x = (vx * cos(self.theta)) * dt
                # delta_y = (vx * sin(self.theta)) * dt

                # TODO: When to update? Before or after?
                self.theta += delta_theta  # Preintegration from the report makes y positive, which seems to be more incorrect

                if (not isnan(delta_x)):
                    self.x = self.x + delta_x  # TODO: Doubts about the sign btw sins 
                if (not isnan(delta_y)):
                    self.y = self.y + delta_y

                # self.x += delta_x
                # self.y += delta_y

                quaternion = tr.quaternion_from_euler(0.0, 0.0, self.theta)

                transform_msg = TransformStamped()
                transform_msg.header.stamp = timestamp
                transform_msg.header.frame_id = "odom"
                transform_msg.child_frame_id = "vehicle"

                transform_msg.transform.translation.x = self.x
                transform_msg.transform.translation.y = self.y
                transform_msg.transform.translation.z = 0.0
                transform_msg.transform.rotation.x = quaternion[0]
                transform_msg.transform.rotation.y = quaternion[1]
                transform_msg.transform.rotation.z = quaternion[2]
                transform_msg.transform.rotation.w = quaternion[3]
                
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

                self.left_count_prev = left_count[i]
                self.right_count_prev = right_count[i]
                self.timestamp_last = timestamp

                # vehicle_world_Rij = (R.from_quat([quaternion[0], quaternion[1], quaternion[2], quaternion[3]])).as_matrix() # qx, qy, qz, qw
                # vehicle_world_tij = np.array([[self.x, self.y, 0.0]]).transpose()
                # vehicle_world_Hij = np.block([vehicle_world_Rij, vehicle_world_tij])
                # vehicle_world_Hij = np.concatenate((vehicle_world_Hij, np.array([[0, 0, 0, 1]])))

                # vehicle_global_Hij = np.dot(world_global_Hij, vehicle_world_Hij)

                # Rij = np.array([[vehicle_global_Hij[0, 0], vehicle_global_Hij[0, 1], vehicle_global_Hij[0, 2]], [vehicle_global_Hij[1, 0], vehicle_global_Hij[1, 1], vehicle_global_Hij[1, 2]], [vehicle_global_Hij[2, 0], vehicle_global_Hij[2, 1], vehicle_global_Hij[2, 2]]])
                # r = R.from_matrix(Rij)
                # q = r.as_quat()
                # print(r.as_matrix())
                # print(q)
                # print("***")

                # self.global_poses[i, 0] = time_usec / 1e9
                # self.global_poses[i, 1] = odom_msg.pose.pose.position.x
                # self.global_poses[i, 2] = odom_msg.pose.pose.position.y
                # self.global_poses[i, 3] = odom_msg.pose.pose.position.z
                # self.global_poses[i, 4] = odom_msg.pose.pose.orientation.x
                # self.global_poses[i, 5] = odom_msg.pose.pose.orientation.y
                # self.global_poses[i, 6] = odom_msg.pose.pose.orientation.z
                # self.global_poses[i, 7] = odom_msg.pose.pose.orientation.w

                # self.global_poses[i, 0] = time_usec
                # self.global_poses[i, 1] = vehicle_global_Hij[0, 0]  # TODO: Check how the components are being extracted in the case first
                # self.global_poses[i, 2] = vehicle_global_Hij[0, 1]
                # self.global_poses[i, 3] = vehicle_global_Hij[0, 2]
                # self.global_poses[i, 4] = vehicle_global_Hij[0, 3]
                # self.global_poses[i, 5] = vehicle_global_Hij[1, 0]
                # self.global_poses[i, 6] = vehicle_global_Hij[1, 1]
                # self.global_poses[i, 7] = vehicle_global_Hij[1, 2]
                # self.global_poses[i, 8] = vehicle_global_Hij[1, 3]
                # self.global_poses[i, 9] = vehicle_global_Hij[2, 0]
                # self.global_poses[i, 10] = vehicle_global_Hij[2, 1]
                # self.global_poses[i, 11] = vehicle_global_Hij[2, 2]
                # self.global_poses[i, 12] = vehicle_global_Hij[2, 3]

                self.bag.write('odom', odom_msg, t=timestamp)
                self.bag.write('tf_wheels', transform_msg, t=timestamp)

        finally:

            self.bag.close()
            # np.savetxt("odom.csv", self.global_poses, delimiter=' ')

        return 0

if __name__ == '__main__':
    converter = Converter(sys.argv)
    sys.exit(converter.main())

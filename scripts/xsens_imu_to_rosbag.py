# !/usr/bin/python
#
# Convert the ms25 csv files to a rosbag
#
# Running:
#
#   python3 ms25_to_rosbag.py ms25.csv ms25.bag
#

import rosbag, rospy
from sensor_msgs.msg import Imu

import sys
import numpy as np

from time import sleep

def main(args):

    time_factor = 1e9  # ns -> s
    interp_delta_t = 0.005

    def interp_data(x, t, t0):

        x_int = np.zeros((t.shape[0], x.shape[1]))

        for i in range(0, x.shape[1]):
            x_int[:, i] = np.interp(t, (x[:, 0] - t0) / time_factor, x[:, i])

        return x_int

    if len(sys.argv) < 2:
        print("Please specify xsense imu file")
        return 1

    if len(sys.argv) < 3:
        print("Please, specify output rosbag file")
        return 1

    xsens_imu = np.loadtxt(sys.argv[1], delimiter=",")

    file_bag = sys.argv[2]
    bag = rosbag.Bag(file_bag, 'w')

    t0 = xsens_imu[0, 0]
    t_end = xsens_imu[-1, 0]

    # Interpolate all
    # Transform differential measurement into integrated measurement
    t_end = int((t_end - t0) / time_factor)
    t = np.linspace(0, t_end, num=int(t_end / interp_delta_t))

    xsens_imu = interp_data(xsens_imu, t, t0)
    # np.savetxt("test_interpolation.csv", self.wheels, delimiter=",")

    times_usec = xsens_imu[:, 0]

    accel_xs = xsens_imu[:, 11]
    accel_ys = xsens_imu[:, 12]
    accel_zs = xsens_imu[:, 13]

    ang_vel_rs = xsens_imu[:, 8]
    ang_vel_ps = xsens_imu[:, 9]
    ang_vel_hs = xsens_imu[:, 10]

    try:

        for i, time_usec in enumerate(times_usec):

            print("Processing timestamp: " + str(time_usec))

            timestamp = rospy.Time.from_sec(time_usec / 1e9)

            imu_msg = Imu()
            imu_msg.header.stamp = timestamp
            imu_msg.header.frame_id = "xsens_imu"

            imu_msg.linear_acceleration.x = accel_xs[i]
            imu_msg.linear_acceleration.y = accel_ys[i]
            imu_msg.linear_acceleration.z = accel_zs[i]

            imu_msg.angular_velocity.x = ang_vel_rs[i]
            imu_msg.angular_velocity.y = ang_vel_ps[i]
            imu_msg.angular_velocity.z = ang_vel_hs[i]

            bag.write('xsens_imu/data', imu_msg, t=timestamp)

    finally:

        bag.close()

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv))

# !/usr/bin/python
#
# Convert the wheel encoders csv files to a rosbag
#
# Running:
#
#   python3 encoder_to_rosbag.py encoder.csv encoder.bag
#

import rosbag, rospy
from kaist_tools.msg import EncoderTick

import sys
import numpy as np


class Converter:
    def __init__(self, args):
        if len(args) < 2:
            print("Please specify encoders file")
            return 1

        if len(args) < 3:
            print("Please, specify output rosbag file")
            return 1

        self.wheels = np.loadtxt(args[1], delimiter=',')

        self.file_bag = args[2]

        self.bag = rosbag.Bag(self.file_bag, 'w')

        self.time_factor = 1e9  # ns -> s
        self.interp_delta_t = 0.005

    def interp_data(self, x, t, t0):

        x_int = np.zeros((t.shape[0], x.shape[1]))

        for i in range(0, x.shape[1]):
            x_int[:, i] = np.interp(t, (x[:, 0] - t0) / self.time_factor, x[:, i])

        return x_int

    def main(self):

        t0 = self.wheels[0, 0]
        t_end = self.wheels[-1, 0]

        # Interpolate all
        # Transform differential measurement into integrated measurement
        t_end = int((t_end - t0) / self.time_factor)
        t = np.linspace(0, t_end, num=int(t_end / self.interp_delta_t))

        self.wheels = self.interp_data(self.wheels, t, t0)
        # np.savetxt("test_interpolation.csv", self.wheels, delimiter=",")

        times_usec = self.wheels[:, 0]

        left_count = self.wheels[:, 1]
        right_count = self.wheels[:, 2]

        try:

            for i, time_usec in enumerate(times_usec):

                print("Processing timestamp: " + str(time_usec))

                timestamp = rospy.Time.from_sec(time_usec / 1e9)

                encoder_tick_msg = EncoderTick()
                encoder_tick_msg.header.stamp = timestamp
                encoder_tick_msg.left_tick.data = left_count[i]
                encoder_tick_msg.right_tick.data = right_count[i]

                self.bag.write('encoder_ticks', encoder_tick_msg, t=timestamp)

        finally:

            self.bag.close()

        return 0


if __name__ == '__main__':
    rospy.init_node("encoder_bag_creator")

    converter = Converter(sys.argv)
    sys.exit(converter.main())

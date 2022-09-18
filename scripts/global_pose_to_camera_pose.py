import numpy as np
import sys

import rosbag, rospy
from nav_msgs.msg import Odometry
import tf.transformations as tr
from geometry_msgs.msg import TransformStamped

from scipy.spatial.transform import Rotation as R

class Converter:
    def __init__(self, args):
        if len(args) < 2:
            print("Please specify input file")
            return 1

        if len(args) < 3:
            print("Please specify output txt file")
            return 1

        # if len(args) < 4:
        #     print("Please specify output bag file")
        #     return 1
        
        self.data_input = np.loadtxt(args[1], delimiter=',')

        self.filename_output = args[2]

        self.data_output = np.empty((self.data_input.shape[0], 8))

        # self.file_bag = args[3]
        
        # self.bag = rosbag.Bag(self.file_bag, 'w')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def main(self):

        try:

            times_usec = self.data_input[:, 0]

            r_00 = self.data_input[:, 1]
            r_01 = self.data_input[:, 2]
            r_02 = self.data_input[:, 3]
            x = self.data_input[:, 4]

            r_10 = self.data_input[:, 5]
            r_11 = self.data_input[:, 6]
            r_12 = self.data_input[:, 7]
            y = self.data_input[:, 8]

            r_20 = self.data_input[:, 9]
            r_21 = self.data_input[:, 10]
            r_22 = self.data_input[:, 11]
            z = self.data_input[:, 12]

            # Urban 29
            # world_global_Rij = np.array([[-0.9986975182, -0.02340548807, -0.04533707308],
            #                              [0.02381712338, -0.9996796793, -0.008560570362],
            #                              [-0.04512218635, -0.009629219038, 0.9989350662]])
            # world_global_tij = np.array([[332538.7257, 4136123.762, 17.98436895]]).transpose()

            # Urban 21
            # world_global_Rij = np.array([[0.0127029128, -0.9998056982, 0.01507321485], [0.9996953553, 0.0130176563, 0.02096991422], [-0.02116205765, 0.01480224388, 0.9996664748]])
            # world_global_tij = np.array([[338677.8155, 4086542.602, 18.48196375]]).transpose()

            # Urban 26
            world_global_Rij = np.array([[-0.9997840965, -0.0193252349, 0.007635158537],
                                         [0.01942543815, -0.9997231711, 0.01327529918],
                                         [0.007376496629, 0.0134207493, 0.9998827285]])
            world_global_tij = np.array([[330089.6744, 4118151.702, 18.03757139]]).transpose()
            
            world_global_Hij = np.block([world_global_Rij, world_global_tij])
            world_global_Hij = np.concatenate((world_global_Hij, np.array([[0, 0, 0, 1]])))

            global_world_Hij = np.linalg.inv(world_global_Hij)

            stereo_left_vehicle_Rij = np.array([[-0.00680499, -0.0153215, 0.99985],
                                                [-0.999977, 0.000334627, -0.00680066],
                                                [-0.000230383, -0.999883, -0.0153234]])
            stereo_left_vehicle_tij = np.array([[1.64239, 0.247401, 1.58411]]).transpose()

            stereo_left_vehicle_Hij = np.block([stereo_left_vehicle_Rij, stereo_left_vehicle_tij])
            stereo_left_vehicle_Hij = np.concatenate((stereo_left_vehicle_Hij, np.array([[0, 0, 0, 1]])))

            for i, time_usec in enumerate(times_usec):

                print("Processing timestamp: " + str(time_usec))

                timestamp = rospy.Time.from_sec(time_usec / 1e9)

                # Rij = np.array([[r_00[i], r_01[i], r_02[i]], [r_10[i], r_11[i], r_12[i]], [r_20[i], r_21[i], r_22[i]]])
                # r = R.from_matrix(Rij)
                # q = r.as_quat()

                # Stereo_left -> World

                vehicle_global_Hij = np.array([[r_00[i], r_01[i], r_02[i], x[i]], [r_10[i], r_11[i], r_12[i], y[i]], [r_20[i], r_21[i], r_22[i], z[i]], [0.0, 0.0, 0.0, 1.0]])
                vehicle_world_Hij = np.dot(global_world_Hij, vehicle_global_Hij)

                stereo_left_world_Hij = np.dot(vehicle_world_Hij, stereo_left_vehicle_Hij)

                Rij = np.array([[stereo_left_world_Hij[0, 0], stereo_left_world_Hij[0, 1], stereo_left_world_Hij[0, 2]],
                                [stereo_left_world_Hij[1, 0], stereo_left_world_Hij[1, 1], stereo_left_world_Hij[1, 2]],
                                [stereo_left_world_Hij[2, 0], stereo_left_world_Hij[2, 1], stereo_left_world_Hij[2, 2]]])
                r = R.from_matrix(Rij)
                q = r.as_quat()
                print(r.as_matrix())
                print(q)
                print("***")

                self.data_output[i, 0] = time_usec / 1e9
                self.data_output[i, 1] = stereo_left_world_Hij[0, 3]
                self.data_output[i, 2] = stereo_left_world_Hij[1, 3]
                self.data_output[i, 3] = stereo_left_world_Hij[2, 3]
                self.data_output[i, 4] = q[0]
                self.data_output[i, 5] = q[1]
                self.data_output[i, 6] = q[2]
                self.data_output[i, 7] = q[3]

                transform_msg = TransformStamped()
                transform_msg.header.stamp = timestamp
                transform_msg.header.frame_id = "odom"
                transform_msg.child_frame_id = "gt"

                transform_msg.transform.translation.x = stereo_left_world_Hij[0, 3]
                transform_msg.transform.translation.y = stereo_left_world_Hij[1, 3]
                transform_msg.transform.translation.z = stereo_left_world_Hij[2, 3]
                transform_msg.transform.rotation.x = q[0]
                transform_msg.transform.rotation.y = q[1]
                transform_msg.transform.rotation.z = q[2]
                transform_msg.transform.rotation.w = q[3]
                
                odom_msg = Odometry()
                odom_msg.header.stamp = timestamp
                odom_msg.header.frame_id = "gt"

                odom_msg.pose.pose.position.x = stereo_left_world_Hij[0, 3]
                odom_msg.pose.pose.position.y = stereo_left_world_Hij[1, 3]
                odom_msg.pose.pose.position.z = stereo_left_world_Hij[2, 3]
                odom_msg.pose.pose.orientation.x = q[0]
                odom_msg.pose.pose.orientation.y = q[1]
                odom_msg.pose.pose.orientation.z = q[2]
                odom_msg.pose.pose.orientation.w = q[3]

                # self.bag.write('odom_gt', odom_msg, t=timestamp)
                # self.bag.write('tf_wheels', transform_msg, t=timestamp)

        finally:

            # self.bag.close()
            np.savetxt(self.filename_output, self.data_output, delimiter=' ')
        
        return 0


if __name__ == '__main__':
    converter = Converter(sys.argv)
    sys.exit(converter.main())
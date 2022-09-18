# from scipy.spatial.transform import Rotation as R
import numpy as np
from scipy.spatial.transform import Rotation as R

#
# Get extrinsic
#
# Format:
#   child_frame -> base_frame
#

# IMU -> Vehicle 
imu_vehicle_Rij = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
print(imu_vehicle_Rij)

imu_vehicle_tij = np.array([[-0.07, 0, 1.7]]).transpose()
print(imu_vehicle_tij)

imu_vehicle_Hij = np.block([imu_vehicle_Rij, imu_vehicle_tij])
imu_vehicle_Hij = np.concatenate((imu_vehicle_Hij, np.array([[0, 0, 0, 1]])))
print(imu_vehicle_Hij)

# Stereo_Left -> Vehicle
stereo_left_vehicle_Rij = np.array([[-0.00680499, -0.0153215, 0.99985], [-0.999977, 0.000334627, -0.00680066], [-0.000230383, -0.999883, -0.0153234]])
print(stereo_left_vehicle_Rij)

stereo_left_vehicle_tij = np.array([[1.64239, 0.247401, 1.58411]]).transpose()
print(stereo_left_vehicle_tij)

stereo_left_vehicle_Hij = np.block([stereo_left_vehicle_Rij, stereo_left_vehicle_tij])
stereo_left_vehicle_Hij = np.concatenate((stereo_left_vehicle_Hij, np.array([[0, 0, 0, 1]])))
print(stereo_left_vehicle_Hij)

# Stereo_Left -> IMU
stereo_left_imu_Hij = np.dot(np.linalg.inv(imu_vehicle_Hij), stereo_left_vehicle_Hij)
print(stereo_left_imu_Hij)
print("Finished!")


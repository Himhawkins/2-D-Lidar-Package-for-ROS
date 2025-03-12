#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R

def define_frame_lidar(ax, ay, az, tilt_x, tilt_y):
    # Normalize accelerometer data to get gravity vector
    gravity_vector = np.array([ax, ay, az])
    gravity_vector /= np.linalg.norm(gravity_vector)

    # Compute rotation matrices from tilt angles
    rotation_x = R.from_euler('x', tilt_x, degrees=True).as_matrix()
    rotation_y = R.from_euler('y', tilt_y, degrees=True).as_matrix()

    # Combine rotations
    rotation_matrix = rotation_y @ rotation_x

    # Transform gravity vector into Lidar frame
    lidar_frame = rotation_matrix @ gravity_vector

    return lidar_frame

def publish_rotating_lidar_frame(ax, ay, az, tilt_x, tilt_y):
    rospy.init_node('lidar_frame_publisher')
    broadcaster = TransformBroadcaster()
    rate = rospy.Rate(100)  # 100 Hz

    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        # Compute elapsed time and rotation angle (1 revolution per second)
        elapsed_time = rospy.Time.now().to_sec() - start_time
        angle = (360 * elapsed_time) % 360  # Angle in degrees

        # Define base Lidar frame using accelerometer and tilt data
        lidar_frame = define_frame_lidar(ax, ay, az, tilt_x, tilt_y)

        # Apply dynamic rotation around x-axis
        rotation_dynamic = R.from_euler('x', angle, degrees=True).as_matrix()
        rotated_frame = rotation_dynamic @ lidar_frame.reshape(3, 1)

        # Publish transform for visualization in RViz
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"
        transform.child_frame_id = "lidarddd"
        
        # Set translation (example: origin at [0, 0, 0])
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        
        # Set rotation as quaternion
        quaternion = R.from_matrix(rotation_dynamic).as_quat()
        transform.transform.rotation.x = quaternion[2]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[0]
        transform.transform.rotation.w = quaternion[3]

        broadcaster.sendTransform(transform)
        
        rate.sleep()

if __name__ == "__main__":
    ax, ay, az = 0.0, 0.0, -9.81  # Example accelerometer data (gravity vector)
    tilt_x, tilt_y = 10.0, 15.0   # Example tilt angles in degrees
    publish_rotating_lidar_frame(ax, ay, az, tilt_x, tilt_y)

#!/usr/bin/env python3

import rospy
import tf
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
import numpy as np
import time
import math
# Initialize state variables
x = np.zeros((6,))  # Initial state vector: [x, y, vx, vy, ax, ay]
P = np.eye(6)  # Initial covariance matrix
F = np.eye(6)  # State transition matrix (to be updated dynamically)
Q = np.eye(6) * 0.1  # Process noise covariance
R = np.eye(2) * 0.1  # Measurement noise covariance
H = np.array([[0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]])  # Observation matrix

last_time = None

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to a quaternion.
    """
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(*q)


def serial_data_callback(msg):
    """
    Callback function to process the serial_data topic and publish odometry.
    """
    # Extract required data from the message
    angle_x = msg.data[2]
    angle_y = msg.data[3]
    angle_z = msg.data[4]
    acc_x = msg.data[5]
    acc_y = msg.data[6]
    acc_z = msg.data[7]

    # Compute velocity from acceleration (assuming constant time step for simplicity)
    #global x, y, th, last_time
    global x, P, F, last_time, old_angle,ang_Zz
    current_time = rospy.Time.now()
    
    dt = (current_time - last_time).to_sec()
    last_time = current_time
    
     
    ang_Z= -1*math.radians(msg.data[1])	
    if ang_Z != old_angle:
        ang_Zz=1*math.radians(msg.data[1])-math.radians(45)
    else:
    	ang_Zz=ang_Zz + math.radians(6) 
    old_angle=	ang_Z	
    ang_Zz=0	
    #vx = acc_x * dt
    #vy = acc_y * dt
    #vth = acc_z * dt

    # Update position based on velocity
    #x += vx * dt
    #y += vy * dt
    #th += vth * dt
    
    #KALMAN IMPLEMENTATION
    F[0][2] = F[1][3] = dt
    F[2][4] = F[3][5] = dt
    # Prediction step
    x_pred = F @ x
    P_pred = F @ P @ F.T + Q
    # Update step using accelerometer data as measurement
    z = np.array([acc_x, acc_y])  # ax and ay from accelerometer
    y = z - H @ x_pred
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    x = x_pred + K @ y
    P = (np.eye(len(K)) - K @ H) @ P_pred
    print(f"Estimated Position: x={x[0]}, y={x[1]}")
    # Create quaternion from Euler angles
  
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "world"
    transform.child_frame_id = "lidarddd"#"laser_frame"
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    quaternion = euler_to_quaternion(3.14+math.radians(angle_x), math.radians(angle_y), 0)#angle_z)
    transform.transform.rotation = quaternion
    transform2 = TransformStamped()
    transform2.header.stamp = rospy.Time.now()
    transform2.header.frame_id = "lidarddd"
    transform2.child_frame_id = "lidar_viss"#"laser_frame"
    transform2.transform.translation.x = 0.0
    transform2.transform.translation.y = 0.0
    transform2.transform.translation.z = 0.0
    quaternion2 = euler_to_quaternion(3.14+math.radians(angle_x), math.radians(angle_y), ang_Zz)#angle_z)
    transform2.transform.rotation = quaternion2
    print(quaternion)
    #transform.transform.rotation.z=0
    #transform.transform.rotation.x = quaternion[0]
    #transform.transform.rotation.y = quaternion[1]
    #transform.transform.rotation.z = quaternion[2]
    #transform.transform.rotation.w = quaternion[3]	 	

    # Publish the transform over tf
    odom_broadcaster.sendTransform(transform)
    odom_broadcaster.sendTransform(transform2)

    # Create and populate the Odometry message
    odom_msg = Odometry()
    odom_msg.header.stamp = current_time
    odom_msg.header.frame_id = "odom"
    
    # Set position and orientation in the message
    odom_msg.pose.pose.position.x = 0#x
    odom_msg.pose.pose.position.y = 0#y
    odom_msg.pose.pose.position.z = 0.0
    odom_msg.pose.pose.orientation = quaternion

    # Set velocity in the message
    odom_msg.child_frame_id = "lidarddd"#"laser_frame"
    odom_msg.twist.twist.linear.x = 0#vx
    odom_msg.twist.twist.linear.y = 0#vy
    odom_msg.twist.twist.angular.z = 0# EARLIER: vth

    # Publish the message
    odom_pub.publish(odom_msg)
    print(odom_msg)

if __name__ == "__main__":
    rospy.init_node('serial_to_odom', anonymous=True)

    # Initialize global variables for position and time tracking
    #global last_time
    last_time = rospy.Time.now()
    old_angle=None	
    ang_Zz=None
    # Create publisher for Odometry messages
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

    # Create TF broadcaster for transform publishing
    odom_broadcaster =TransformBroadcaster()

    # Subscribe to the serial_data topic (assuming it publishes std_msgs/Float32MultiArray)
    rospy.Subscriber("serial_data", Float32MultiArray, serial_data_callback)

    rospy.spin()


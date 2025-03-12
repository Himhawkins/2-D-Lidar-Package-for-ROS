#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
import numpy as np
import math



class LaserScanPublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('laser_scan_publisher', anonymous=True)

        # Subscriber and Publisher
        self.sub = rospy.Subscriber('serial_data', Float32MultiArray, self.serial_data_callback)
        self.pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        #self.pub2 = rospy.Publisher('/scan_lin', LaserScan, queue_size=10)
        # Data storage
        self.data_array = []  # Stores distance and angle for current range
        self.current_angle_range = None  # Tracks the current 45-degree range
	
    def serial_data_callback(self, msg):
        # Extract distance and angle from incoming message
        distance = msg.data[0] -5 #OFFSET
        angle = msg.data[1]
        #print("DATA READ!: ",msg.data)
        # Determine the 45-degree range of the current angle
        new_angle_range = int(angle // 45) * 45

        # If this is a new angle range, process and publish the previous range
        if self.current_angle_range is not None and new_angle_range != self.current_angle_range:
            self.process_and_publish()

            # Reset the array for the new angle range
            self.data_array = []

        # Update current angle range and append new data
        self.current_angle_range = new_angle_range
        self.data_array.append((distance, angle))
        #print(self.data_array)
    def process_and_publish(self):
        if not self.data_array:
            return

        # Prepare LaserScan message
        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = "lidarddd"

        scan_msg.angle_min = np.radians(self.current_angle_range)
        scan_msg.angle_max = np.radians(self.current_angle_range + 45)
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / len(self.data_array)
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min =  .01#0.05  # Minimum valid range
        scan_msg.range_max = 1000#10.0  # Maximum valid range

        # Linearize distances across the angle range
        distances = [d/100 for d, _ in self.data_array]
        
        #ax=msg.data[2]
        #ay=msg.data[3]
        #az=self.current_angle_range
        #distances2 = [(calculate_norm(ax,ay,az, d))/100 for d, _ in self.data_array]
        scan_msg.ranges = distances
        self.pub.publish(scan_msg)
        #scan_msg.ranges = distances2
        #self.pub2.publish(scan_msg)
        print("PUBLISHED: ",scan_msg)
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LaserScanPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass


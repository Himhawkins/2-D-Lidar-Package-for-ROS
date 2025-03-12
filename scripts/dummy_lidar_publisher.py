#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
import time

def publish_laser_data():
    # Initialize the ROS node
    rospy.init_node('dummy_lidar_publisher', anonymous=True)

    # Create a publisher for LaserScan messages
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

    # Set the publishing rate
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Create a LaserScan message
        scan = LaserScan()

        # Fill in the header
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "laser_frame"

        # Define scan parameters
        scan.angle_min = -math.pi / 2  # Start angle (-90 degrees)
        scan.angle_max = math.pi / 2   # End angle (+90 degrees)
        scan.angle_increment = math.pi / 180  # Angular resolution (1 degree)
        scan.time_increment = 0.0  # Time between measurements (not used here)
        scan.scan_time = 0.1       # Time between scans (10 Hz)
        scan.range_min = 0.2       # Minimum range of the sensor (meters)
        scan.range_max = 10.0      # Maximum range of the sensor (meters)

        # Generate dummy range values (e.g., sine wave pattern)
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        ranges = []
        for i in range(num_readings):
            angle = scan.angle_min + i * scan.angle_increment
            ranges.append(5.0 + math.sin(angle))  # Example: sine wave pattern

        scan.ranges = ranges
        scan.intensities = []
        print(scan)
        pub.publish(scan)
        
        rospy.loginfo("Published dummy LaserScan data")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_laser_data()
    except rospy.ROSInterruptException:
        pass

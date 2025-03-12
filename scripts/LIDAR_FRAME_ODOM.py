#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion
import numpy as np
def random_unit_quaternion():
    u1, u2, u3 = np.random.rand(3)
    q1 = np.sqrt(1 - u1) * np.sin(2 * np.pi * u2)
    q2 = np.sqrt(1 - u1) * np.cos(2 * np.pi * u2)
    q3 = np.sqrt(u1) * np.sin(2 * np.pi * u3)
    q4 = np.sqrt(u1) * np.cos(2 * np.pi * u3)
    return (q1, q2, q3, q4)


def handle_odom():#msg): UNCOMMENT THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # Extract position and orientation from odometry message
    #position = msg.pose.pose.position   AND THESE TWO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    #orientation = msg.pose.pose.orientation
    
    # Create a TransformBroadcaster
    br = tf.TransformBroadcaster()
    
    # Broadcast the transform
    #br.sendTransform(
     #   (position.x, position.y, position.z),
     #   (orientation.x, orientation.y, orientation.z, orientation.w),
     #   rospy.Time.now(),
     #   "laser_frame",  # Child frame
     #   "world"         # Parent frame
   # )
    
      # Broadcast the transform
    for i in range(30):
	    br.sendTransform(
	       (0,0,0),
	       random_unit_quaternion(),
	       rospy.Time.now(),
	       "world",  # Child frame
	       "laser_frame"         # Parent frame
	    )
	    
	    print("TF SENT!")

def main():
    rospy.init_node('laser_tf_broadcaster')
    
    # Subscribe to the /odom topic
    handle_odom() #UNCOMMMENT BELOW!!!!!!!!!!!!!!!
    #rospy.Subscriber('/odom', Odometry, handle_odom) # but odom is published when signal is published, so 
    #this would publish after that. BUT, laser data is alsp published AFTER signal is published.
    #So Some how, this timing and laser data are being published at different times? IS that it?
    #delay(10)
    #rospy.spin()

if __name__ == '__main__':
    while 1:
    	main()

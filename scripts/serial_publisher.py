#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import serial
import numpy as np
from std_msgs.msg import Float32MultiArray
encoding= 'ascii' #intially this was utf-8
def convert_to_signed(value):
    # Convert a 16-bit unsigned integer to signed (-32,768 to 32,767)
    if value > 0x7FFF:  # If the value exceeds the maximum for signed 16-bit integers
        return value - 0x10000
    return value

def Process_and_Package(data):
    # Ensure data is exactly 18 bytes
    data=[89,89] +data
    #print("YOYOYOY data is:",data)
    if len(data) != 18:
        raise ValueError("Data must be exactly 18 bytes long")

    # Check if the first two bytes are 89, 89
    if data[0] != 89 or data[1] != 89:
        raise ValueError("First two bytes must be 89, 89 actual:",data)

    # Calculate distance from byte 3 and byte 4
    distance = (data[3] * 256) + data[2]

    # Extract angle from byte 5
    angle = data[4] +data[5]*256

    # Calculate angleX, angleY, angleZ, accX, accY, accZ
    angleX = convert_to_signed(data[6] + (data[7] <<8)) / 100.0
    # Calculate angleY, angleZ, accX, accY, accZ normally
    angleY = convert_to_signed(data[8] + (data[9] <<8)) / 100.0
    angleZ = convert_to_signed(data[10] + (data[11] <<8)) / 100.0
    accX = convert_to_signed(data[12] + (data[13] <<8)) / 100.0
    accY = convert_to_signed(data[14] + (data[15] <<8)) / 100.0
    accZ = convert_to_signed(data[16] + (data[17] <<8)) / 100.0
   # Return results in a list

    return [distance,angle,angleX,angleY, angleZ, accX,accY,accZ]

def serial_publisher():
    # Initialize the ROS node
	rospy.init_node('serial_publisher', anonymous=True)

	# Create a publisher on the 'serial_data' topic
	pub = rospy.Publisher('serial_data', Float32MultiArray, queue_size=2)
	#NOTEOTEOT :: EDIT QUEUE TO ADJUST LAG

	# Set the loop rate (10 Hz)
	rate = rospy.Rate(100)

    # Open the serial port (adjust '/dev/ttyACM0' and baudrate as needed)
	try:
		ser = serial.Serial('/dev/ttyUSB2', 9600, timeout=1)
		rospy.loginfo("Serial port opened successfully")
	except serial.SerialException as e:
		rospy.logerr(f"Failed to open serial port: {e}")
		return
    
	while not rospy.is_shutdown():
		try:
		    # Read data from the serial port
				t=ser.read(1)
				#rospy.loginfo(f"Raw first: {t}")
				if t==b'Y':
					#rospy.loginfo("READ FIRST!")
					t==ser.read(1)
					if t==b'Y':
						#rospy.loginfo("READ Second!")
						line = ser.read(16)
						integers = ([byte for byte in line])
						#rospy.loginfo(f"RAW: {line}")
						rospy.loginfo(f"Read from serial: {Process_and_Package(integers)}")
						msg = Float32MultiArray()
						msg.data = Process_and_Package(integers)
						# Publish the data to the topic
						pub.publish(msg)
		except Exception as e:
		    rospy.logerr(f"Error reading from serial port: {e}")
        
		rate.sleep()

if __name__ == '__main__':
    try:
        serial_publisher()
    except rospy.ROSInterruptException:
        pass

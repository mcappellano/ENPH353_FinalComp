#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import time

# Define global variables
current_frame = None
bridge = CvBridge()
current_encoding = [0, 1, 0]  # Default center encoding

# Directory to save images
image_save_dir = "/home/fizzer/ros_ws/src/Track_images_V1"
os.makedirs(image_save_dir, exist_ok=True)

# Define adjustable parameters
THRESHOLD = 120  # Binary threshold for detecting the line
SLICE_HEIGHTS = [0.60, 0.75, 0.9]  # Percentages of the image height for slices
LINEAR_SPEED = 2  # Base linear speed
TURNING_SPEED = 5  # Angular speed for corrections
CENTER_TOLERANCE = 10  # Tolerance to consider the line "centered"

def save_image_with_one_hot_encoding(one_hot_encoding):
    global current_frame
    if current_frame is None:
        rospy.logwarn("No image frame received yet.")
        return

    # Get the current timestamp
    timestamp = time.time()

    # Format the filename using timestamp and one-hot encoding
    filename = os.path.join(image_save_dir, f"{timestamp:.6f}_{one_hot_encoding}.png")
    
    rospy.loginfo(f"Saving image: {filename}")  # Add this log to track saving process

    # Save the image
    cv2.imwrite(filename, current_frame)
    rospy.loginfo(f"Saved image: {filename}")

def get_one_hot_encoding_from_twist(msg):
    """
    Generate a one-hot encoding based on angular velocity (turning direction).
    """
    if msg.angular.z < 0:
        return [1, 0, 0]  # Left turn
    elif msg.angular.z == 0:
        return [0, 1, 0]  # Center
    else:
        return [0, 0, 1]  # Right turn

def cmd_vel_callback(msg):
    # Update the current encoding from the received twist message
    global current_encoding
    current_encoding = get_one_hot_encoding_from_twist(msg)
    rospy.loginfo(f"Received twist command, current encoding: {current_encoding}")

def rosImage_2_OpenCV(msg):
    """
    Converts the ROS Image message to OpenCV format and saves it with the current one-hot encoding.
    """
    global current_frame
    try:
        # Convert the ROS Image message to OpenCV format
        current_frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        rospy.loginfo("Image frame received and converted to OpenCV format.")
    except CvBridgeError as e:
        rospy.logerr(f"Failed to convert image: {e}")
        return

    # Save the image with the current one-hot encoding
    save_image_with_one_hot_encoding(current_encoding)

def track_teleop_commands():
    rospy.init_node('teleop_cmd_tracker', anonymous=True)

    # Subscribe to /cmd_vel topic to get teleop commands
    rospy.Subscriber('/B1/cmd_vel', Twist, cmd_vel_callback)
    
    # Subscribe to camera image topic to get images
    rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, rosImage_2_OpenCV)
    
    rospy.loginfo("Tracking teleop_twist_keyboard commands...")
    rospy.spin()

def main():
    rospy.init_node('drive_commands', anonymous=True, log_level=rospy.INFO)

    # Publisher and subscriber for cmd_vel and image topics are handled by track_teleop_commands
    
    rospy.sleep(1.0)
    rate = rospy.Rate(5)  # Publish rate for loop

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        track_teleop_commands()
    except rospy.ROSInterruptException:
        pass

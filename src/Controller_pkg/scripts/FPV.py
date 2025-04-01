#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Define global variables
current_frame = None
bridge = CvBridge()

def rosImage_2_OpenCV(frame):
    """
    Callback function to convert ROS Image message to OpenCV format,
    resize it to 1080x1080, and display it in a window.
    """
    global current_frame
    try:
        # Convert ROS Image message to OpenCV format
        current_frame = bridge.imgmsg_to_cv2(frame, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr(f"Image was not successfully converted: {e}")
        return

    # Resize the image to 1080x1080
    current_frame = cv2.resize(current_frame, (1080, 1080))

    # Display the frame in an OpenCV window
    cv2.imshow("FPV", current_frame)
    cv2.moveWindow("FPV", 0, 0)
    cv2.waitKey(1)  # Necessary for OpenCV to update the window

def main():
    """
    Main function to initialize the ROS node and set up the subscriber for camera images.
    """
    rospy.init_node('drive_commands', anonymous=True, log_level=rospy.INFO)

    # Subscribe to the camera topic to get the image feed
    rospy.Subscriber('B1/rrbot/camera1/image_raw', Image, rosImage_2_OpenCV)

    # Rate control for ROS loop
    rate = rospy.Rate(10)

    # ROS loop to keep the subscriber active
    while not rospy.is_shutdown():
        rate.sleep()

    # Close OpenCV windows when ROS is shut down
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        # Close OpenCV windows if the program is interrupted
        cv2.destroyAllWindows()

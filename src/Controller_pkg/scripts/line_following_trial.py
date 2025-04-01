#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Define global variables
current_frame = None
bridge = CvBridge()

# Define adjustable parameters
THRESHOLD = 100  # Binary threshold for detecting the line
SLICE_HEIGHTS = [0.7, 0.8, 0.9]  # Percentages of the image height for slices
LINEAR_SPEED = 6  # Base linear speed
TURNING_SPEED = 6  # Angular speed for corrections
CENTER_TOLERANCE = 100  # Tolerance to consider the line "centered"


def rosImage_2_OpenCV(frame):
    global current_frame
    try:
        current_frame = bridge.imgmsg_to_cv2(frame, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr(f"Image was not successfully converted: {e}")


def find_line_center():
    global current_frame

    if current_frame is None:
        rospy.logwarn("No image frame received yet.")
        return [None, None, None]

    # Convert the image to grayscale and apply a binary threshold
    gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    _, binary_frame = cv2.threshold(gray_frame, THRESHOLD, 255, cv2.THRESH_BINARY)

    # Get the image dimensions
    height, width = binary_frame.shape

    # Store the line centers for each slice
    line_centers = []

    # Analyze each slice
    for slice_height_percentage in SLICE_HEIGHTS:
        # Get the y-coordinate for the slice
        y_coord = int(height * slice_height_percentage)

        # Extract the horizontal slice
        slice_row = binary_frame[y_coord, :]

        # Find the positions of the line (black pixels)
        line_pixels = np.where(slice_row == 0)[0]  # 0 for black pixels (road)

        if line_pixels.size > 0:
            # Calculate the midpoint of the detected line
            line_center = int(np.mean(line_pixels))
            line_centers.append(line_center)

            # For visualization (optional)
            cv2.circle(current_frame, (line_center, y_coord), 5, (0, 0, 255), -1)

        else:
            # No line detected in this slice
            line_centers.append(None)

    # Show the image for debugging (optional)

    cv2.imshow("Line Detection", current_frame)
    cv2.waitKey(1)

    return line_centers


def drive(pub, line_centers):
    twist = Twist()

    if None in line_centers:
        rospy.logwarn("Line not detected in one or more slices. Slowing down and searching...")
        twist.linear.x = 0.0  # Stop linear motion
        twist.angular.z = TURNING_SPEED  # Turn in place to search
        pub.publish(twist)
        return

    # Calculate the deviation from the center for each slice
    image_width = current_frame.shape[1]
    center_of_image = image_width // 2
    deviations = [line_center - center_of_image for line_center in line_centers]

    # Calculate the average deviation (used to set turning direction)
    avg_deviation = np.mean(deviations)

    # Determine linear speed: Slower when the line is not vertically centered
    vertical_alignment = abs(deviations[0]) + abs(deviations[1]) + abs(deviations[2])
    twist.linear.x = max(1, LINEAR_SPEED * (1 - (vertical_alignment / (3 * CENTER_TOLERANCE))))

    # Determine angular speed: Proportional to how far off-center the line is
    if abs(avg_deviation) > CENTER_TOLERANCE:
        twist.angular.z = -TURNING_SPEED * (avg_deviation / center_of_image)
    else:
        twist.angular.z = 0.0  # No turning needed if centered

    pub.publish(twist)


def main():
    rospy.init_node('drive_commands', anonymous=True, log_level=rospy.INFO)

    publisher = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
    subscriber = rospy.Subscriber('B1/rrbot/camera1/image_raw', Image, rosImage_2_OpenCV)

    rospy.sleep(1.0)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        line_centers = find_line_center()
        drive(publisher, line_centers)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
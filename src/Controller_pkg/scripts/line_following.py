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
THRESHOLD = 120  # Binary threshold for detecting the line
SLICE_HEIGHTS = [0.60,0.75,0.9]  # Percentages of the image height for slices
LINEAR_SPEED = 2  # Base linear speed
TURNING_SPEED = 5  # Angular speed for corrections
CENTER_TOLERANCE = 10  # Tolerance to consider the line "centered"


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



    # HSV = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

    # Define HSV range for detecting the grey road (Adjust values for better accuracy)
    lower_grey = np.array([79, 79, 79])   # Lower bound for grey
    upper_grey = np.array([94, 94, 94])  # Upper bound for grey

    # Create a binary mask where the road is white and everything else is black
    mask = cv2.inRange(current_frame, lower_grey, upper_grey)

    # Invert the mask to make the road black and the background white
    mask_inverted = cv2.bitwise_not(mask)

    # # Convert the image to grayscale and apply a binary threshold
    # gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    # _, binary_frame = cv2.threshold(gray_frame, THRESHOLD, 255, cv2.THRESH_BINARY)

    # Get the image dimensions
    height, width = mask_inverted.shape

    # Store the line centers for each slice
    line_centers = []

    # Analyze each slice
    for slice_height_percentage in SLICE_HEIGHTS:
        # Get the y-coordinate for the slice
        y_coord = int(height * slice_height_percentage)

        # Extract the horizontal slice
        slice_row = mask_inverted[y_coord, :]

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
    cv2.imshow("Binary Frame", mask_inverted)
    cv2.waitKey(1)

    return line_centers


# Define global variable to store last known deviation direction
last_known_deviation = 0 # Positive = right, Negative = left

def drive(pub, line_centers):
    global current_frame
    global last_known_deviation

    if current_frame is None:
        rospy.logwarn("No image frame received yet.")
        return [None, None, None]
    
    copy_frame = current_frame.copy()
    
    twist = Twist()

    image_width = current_frame.shape[1]
    center_of_image = image_width // 2

    # Check if ALL slices lost the line
    if all(center is None for center in line_centers):
        rospy.logwarn("Line completely lost! Turning in last known direction...")

        # Turn sharply in the last known direction
        if last_known_deviation > 0:
            twist.angular.z = -TURNING_SPEED * 2  # Increase turn intensity
        elif last_known_deviation < 0:
            twist.angular.z = TURNING_SPEED * 2
        else:
            twist.angular.z = TURNING_SPEED * 2  # Default to left

        twist.linear.x = 0.0  # Stop forward motion while searching
        pub.publish(twist)
        return

    # Remove None values from line_centers to avoid errors
    valid_line_centers = [lc for lc in line_centers if lc is not None]
    if not valid_line_centers:
        return

    deviations = [line_center - center_of_image for line_center in valid_line_centers]

    # Compute the average deviation
    avg_deviation = np.mean(deviations)
    last_known_deviation = avg_deviation  # Store last known direction

    abs_deviation = abs(avg_deviation)


    # **🔹 Use a quadratic scaling for turning speed (More aggressive corrections)**
    if abs_deviation > CENTER_TOLERANCE:
        twist.angular.z = -TURNING_SPEED * np.sign(avg_deviation) * ((abs(avg_deviation) / center_of_image) ** 2) * 2
    else:
        twist.angular.z = 0.0  # No turning needed if centered

    # # **🔹 Non-linear speed reduction when turning**
    # abs_deviation = abs(avg_deviation)
    twist.linear.x = LINEAR_SPEED * max(0.1, 1 - (abs_deviation / center_of_image))

    text = "linear speed = "+str(twist.linear.x)+"  Angular speed = "+str(twist.angular.z)+""
    text2 = "last dev = " + str(last_known_deviation)+""


    # Define font, scale, thickness, and color
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 1
    thickness = 2
    color = (0, 0, 255)  # Red in BGR

    # Get image dimensions
    (h, w, _) = copy_frame.shape

    # Get text size
    (text_width, text_height), _ = cv2.getTextSize(text, font, scale, thickness)

    # Calculate position (top right)
    x = w - text_width - 10  # 10-pixel padding from right
    y = text_height + 10  # 10-pixel padding from top

    # Put text on image
    cv2.putText(copy_frame, text, (x, y), font, scale, color, thickness)
    cv2.putText(copy_frame, text2, (x, y+text_height), font, scale, color, thickness)


    # Show the image
    cv2.imshow("Image with Text", copy_frame)
    cv2.waitKey(1)

    pub.publish(twist)


# def drive(pub, line_centers):
#     twist = Twist()


#     if None in line_centers:
#         rospy.logwarn("Line not detected in one or more slices. Slowing down and searching...")
#         twist.linear.x = 0.0  # Stop linear motion
#         twist.angular.z = TURNING_SPEED  # Turn in place to search
#         pub.publish(twist)
#         return

#     # Calculate the deviation from the center for each slice
#     image_width = current_frame.shape[1]
#     center_of_image = image_width // 2
#     deviations = [line_center - center_of_image for line_center in line_centers]

#     # Calculate the average deviation (used to set turning direction)
#     avg_deviation = np.mean(deviations)

#     # Determine linear speed: Slower when the line is not vertically centered
#     vertical_alignment = abs(deviations[0]) + abs(deviations[1]) + abs(deviations[2])
#     twist.linear.x = max(1, LINEAR_SPEED * (1 - (vertical_alignment / (3 * CENTER_TOLERANCE))))

#     # Determine angular speed: Proportional to how far off-center the line is
#     if abs(avg_deviation) > CENTER_TOLERANCE:
#         twist.angular.z = -TURNING_SPEED * (avg_deviation / center_of_image)
#     else:
#         twist.angular.z = 0.0  # No turning needed if centered

#     pub.publish(twist)


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
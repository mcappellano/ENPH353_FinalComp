#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
import message_filters
from geometry_msgs.msg import PoseStamped
import os
import tensorflow as tf

extractFlag = True

clue_id_dict = {
    "SIZE": 1,
    "VICTIM": 2,
    "CRIME": 3,
    "TIME": 4,
    "PLACE": 5,
    "MOTIVE": 6,
    "WEAPON": 7,
    "BANDIT": 8
}

# Configuration
TEAM_NAME = "Ctrl-Alt-Defeat"  # Replace with your actual team name
PASSWORD = "Winners"   # Replace with your team password
TIMER_TOPIC = "/score_tracker"
CLOCK_TOPIC = "/clock"

score_publisher = None
model = None

CHAR_MAP = "ABCDEFOHIJKLMNOPQRSTUVWXYZ0123486759 "

# Global variables
start_time = None
elapsed_time = 0

TIMER_DURATION = rospy.Duration(90)

def siftFunc(frame, img_color):
    #Grayscale the reference image
    img = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)

    #SIFT things
    sift = cv2.SIFT_create()
    kp_image, desc_image = sift.detectAndCompute(img, None)
    index_params = dict(algorithm=0, trees=5)
    search_params = dict()
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    #Grayscale the input (cropped clueboard)
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kp_gray_frame, desc_gray_frame = sift.detectAndCompute(gray_frame, None)
    try:
        #This line raised an error once, dont know why. Just try-except it
        matches = flann.knnMatch(desc_image, desc_gray_frame, k=2)
    except:
        print("error with knnMatch")
        return None

    #Find all the good points in SIFT
    good_points = []
    for m, n in matches:
        if m.distance < 0.6 * n.distance:
            good_points.append(m)

    #Homography!
    if len(good_points) > 10:
        query_pts = np.float32([kp_image[m.queryIdx].pt for m in good_points]).reshape(-1, 1, 2)
        train_pts = np.float32([kp_gray_frame[m.trainIdx].pt for m in good_points]).reshape(-1, 1, 2)
        matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)
        try:
            #Another line that randomly raised an
            inv_trans = np.linalg.pinv(matrix)
        except:
            print("error with inversion matrix")
            return None
        h, w = img.shape
        dst = cv2.warpPerspective(frame, inv_trans, (600,400))

        return dst
    else:
        return None

def clueboardFinder(frame, camera_label):

    try:
        # Convert ROS Image message to OpenCV format
        image = CvBridge().imgmsg_to_cv2(frame, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"Image was not successfully converted: {e}")
        return None
        # 1. Load the image
    if image is None:
        raise FileNotFoundError("Could not read the input image!")

    # 3. Define lower/upper bounds for 'blue'
    #    You may need to adjust these values for your particular image lighting
    lower_blue = np.array([85, 0, 0])   # e.g. H=100, S=100, V=50
    upper_blue = np.array([130, 35, 35]) # e.g. H=140, S=255, V=255

    lower_blue2 = np.array([180, 90, 90])   # e.g. H=100, S=100, V=50
    upper_blue2 = np.array([210, 105, 105]) # e.g. H=140, S=255, V=255

    # 4. Create a mask where the color is within the specified range
    mask = cv2.inRange(image, lower_blue, upper_blue)
    mask2 = cv2.inRange(image, lower_blue2, upper_blue2)

    # # 5. Optional: apply some morphological operations to clean up noise
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # fill small holes
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # remove small spots

    mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, kernel)  # fill small holes
    mask2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, kernel)   # remove small spots

    # 6. Find external contours in the mask
    contours1, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours2, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours1 and not contours2:
       return None

    # 7. Pick the contour with the largest area as our billboard candidate
    largest_area = 0
    largest_contour = None

    for cnt in contours1:
        area = cv2.contourArea(cnt)
        if area > largest_area:
            largest_area = area
            largest_contour = cnt

    for cnt in contours2:
        area = cv2.contourArea(cnt)
        if area > largest_area:
            largest_area = area
            largest_contour = cnt

    return (largest_contour, largest_area, image)

# def start_timer(pub):
#     """Start the timer by publishing a message to /score_tracker."""
#     global start_time
#     start_time = rospy.Time.now()  # Store start time
#     start_msg = String(f"{TEAM_NAME},{PASSWORD},0,NA")
#     pub.publish(start_msg)
#     rospy.loginfo("Timer started!")


# def stop_timer(pub):
#     """Stop the timer by publishing a message to /score_tracker."""
#     stop_msg = String(f"{TEAM_NAME},{PASSWORD},-1,NA")
#     pub.publish(stop_msg)
#     rospy.loginfo("Timer stopped!")

def clueboard_callback(right_frame, left_frame):
    right_largest_contour_area_tuple = clueboardFinder(right_frame, "right")
    left_largest_contour_area_tuple = clueboardFinder(left_frame, "left")

    largest_contour = None
    largest_area = 0
    label = None
    image = None
    if right_largest_contour_area_tuple is None:
        if left_largest_contour_area_tuple is None:
            return
        else:
            largest_area = left_largest_contour_area_tuple[1]
            largest_contour = left_largest_contour_area_tuple[0]
            image = left_largest_contour_area_tuple[2]
            label = "left"
    else:
        if left_largest_contour_area_tuple is None:
            largest_area = right_largest_contour_area_tuple[1]
            largest_contour = right_largest_contour_area_tuple[0]
            image = right_largest_contour_area_tuple[2]
            label = "right"
        else:
            if (right_largest_contour_area_tuple[1] > left_largest_contour_area_tuple[1]):
                largest_area = right_largest_contour_area_tuple[1]
                largest_contour = right_largest_contour_area_tuple[0]
                image = right_largest_contour_area_tuple[2]
                label = "right"
            else:
                largest_area = left_largest_contour_area_tuple[1]
                largest_contour = left_largest_contour_area_tuple[0]
                image = left_largest_contour_area_tuple[2]
                label = "left"

    if image is None:
        return
    referenceImg = cv2.imread('reference2.png')
    if referenceImg is None:
        raise FileNotFoundError("Could not read the input image!")
    imgHeight, imgWidth, channels = image.shape
    # 8. Get the bounding rectangle of that largest contour
    if largest_area > 2500:
        x, y, w, h = cv2.boundingRect(largest_contour)
        offset = 20
        y1 = y-offset if y > offset else y
        y2 = y+h+offset if y+h+offset < imgHeight else y+h

        x1 = x-offset if x > offset else x
        x2 = x+w+offset if x+w+offset < imgWidth else x+w
        # 9. Crop the billboard region from the original image
        cropped_clueboard = image[y1:y2, x1:x2]
        # 10. Display or save the cropped billboard
        _h, _w, _c = cropped_clueboard.shape
        ratio = _w / _h
        transformedImg = siftFunc(cropped_clueboard, referenceImg)
        if transformedImg is None:
            print("no homography")
        else:
            clue_type_img = transformedImg[0:180, 240:600]
            clue_value_img = transformedImg[215:385, 15:585]

            if (ratio > 1.4):
                clue_type = text_extractor(clue_type_img)
                clue_value = text_extractor(clue_value_img)
                print(clue_type + " " + label)
                print(clue_value + " " + label)
                try:
                    clue_id = clue_id_dict[clue_type]
                except:
                    print("Misread the clueboard")
                    cv2.waitKey(1)
                    return
                if score_publisher is None:
                    print("score publisher issue")
                else:
                    score_msg = String(f"{TEAM_NAME},{PASSWORD},{clue_id},{clue_value}")
                    score_publisher.publish(score_msg)
    return

def text_extractor(image):

    if model is None:
        rospy.logwarn("Model is not loaded yet!")
        return ""

    lower_blue = np.array([85, 0, 0])   # e.g. H=100, S=100, V=50
    upper_blue = np.array([210, 35, 35]) # e.g. H=140, S=255, V=255

    mask = cv2.inRange(image, lower_blue, upper_blue)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    char_bboxes = []
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        # Filter out contours that are too small or too large to be characters
        if w < 5 or h < 5:
            continue
        # Optionally set an upper limit for w/h if large areas exist that aren't text
        char_bboxes.append((x, y, w, h))

    # Sort bounding boxes by their x (and y) coordinate to maintain reading order
    char_bboxes = sorted(char_bboxes, key=lambda box: (box[1], box[0]))

    output_dir = "chars_output"
    imgHeight, imgWidth, channels = image.shape
    char_array = []
    for i, (x, y, w, h) in enumerate(char_bboxes):
        offset = 8
        y1 = y-offset-17 if y-17 > offset else y
        y2 = y+h+offset+17 if y+h+offset+17 < imgHeight else y+h

        x1 = x-offset if x > offset else x
        x2 = x+w+offset if x+w+offset < imgWidth else x+w
        char_roi = image[y1:y2, x1:x2]  # crop from the original color image or thresh
        char_processed = preprocess_image(char_roi)
        char_vector = model.predict(char_processed)[0]
        char_index = np.argmax(char_vector)
        print(char_index)
        character = CHAR_MAP[char_index]
        char_array.append(character)
        # out_path = os.path.join(output_dir, f"char_{i}.png")
        # cv2.imwrite(out_path, char_roi)
        # print(f"Saved character {i} to {out_path}")
    return "".join(char_array)

def model_init():
    global model
    model_path = "/home/fizzer/ros_ws/src/Controller_pkg/scripts/my_model.h5"
    model = tf.keras.models.load_model(model_path)
    rospy.loginfo(f"✅ Loaded model from {model_path}")

def preprocess_image(cv_image):
    """Resize, normalize, and add batch/channel dimensions."""
    resized = cv2.resize(cv_image, (128, 128))  # Keeps all 3 channels (BGR)
    normalized = resized.astype(np.float32) / 255.0
    expanded = np.expand_dims(normalized, axis=0)  # Shape: (1, 128, 128, 3)
    return expanded

def clock_callback(msg):
    global score_publisher
    """Updates the elapsed time in the Score Tracker."""
    if start_time is not None:
        global elapsed_time
        elapsed_time = (msg.clock - start_time).to_sec()

        if elapsed_time >= TIMER_DURATION.to_sec():
            stop_msg = String(f"{TEAM_NAME},{PASSWORD},-1,NA")
            score_publisher.publish(stop_msg)
        # rospy.loginfo(f"Elapsed Time: {elapsed_time:.2f} seconds")

def main():
    """
    Main function to initialize the ROS node and set up the subscriber for camera images.
    """
    global score_publisher, start_time
    rospy.init_node('clue_board')

    model_init()
    score_publisher = rospy.Publisher(TIMER_TOPIC, String, queue_size=1)
    right_image_sub = message_filters.Subscriber('B1/rrbot/camera3/image_raw', Image)
    left_image_sub = message_filters.Subscriber('B1/rrbot/camera2/image_raw', Image)

    ts = message_filters.ApproximateTimeSynchronizer(
        [right_image_sub, left_image_sub], 
        queue_size=2,
        slop=0.1
    )
    ts.registerCallback(clueboard_callback)

    # # Subscribe to clock to update elapsed time in Score Tracker
    rospy.Subscriber(CLOCK_TOPIC, Clock, clock_callback)

    print("big test")

    rospy.sleep(1.5)  # Allow time for ROS setup

    start_time = rospy.Time.now()  # Store start time
    start_msg = String(f"{TEAM_NAME},{PASSWORD},0,NA")
    score_publisher.publish(start_msg)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        # Close OpenCV windows if the program is interrupted
        cv2.destroyAllWindows()


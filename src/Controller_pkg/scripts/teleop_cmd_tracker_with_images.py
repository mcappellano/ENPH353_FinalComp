#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time
import pickle
from googleapiclient.discovery import build
from google_auth_oauthlib.flow import InstalledAppFlow
from google.auth.transport.requests import Request
from googleapiclient.http import MediaFileUpload

# === Config ===

image_save_dir = "/home/fizzer/ros_ws/src/Track_images_V1"
os.makedirs(image_save_dir, exist_ok=True)

folder_id = "1wdJenqWDkHyVFWadlrcZl731B9L_Vp5y"  # Replace this with your actual Drive folder ID

SCOPES = ['https://www.googleapis.com/auth/drive.file']
creds = None

# === Global State ===

bridge = CvBridge()
current_frame = None
current_encoding = [0, 1, 0]
saved_images = []
recording_active = True  # Flag to stop processing on shutdown

# === Google Drive Auth ===

def authenticate_google_drive():
    global creds
    if os.path.exists('token.pickle'):
        with open('token.pickle', 'rb') as token:
            creds = pickle.load(token)
    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file('credentials.json', SCOPES)
            creds = flow.run_local_server(port=0)
        with open('token.pickle', 'wb') as token:
            pickle.dump(creds, token)
    return build('drive', 'v3', credentials=creds)

service = authenticate_google_drive()

# === Save and Upload Logic ===

def save_image_with_one_hot_encoding(one_hot_encoding):
    if rospy.is_shutdown() or not recording_active:
        return
    global current_frame, saved_images
    if current_frame is None:
        return
    timestamp = time.time()
    filename = os.path.join(image_save_dir, f"{timestamp:.6f}_{one_hot_encoding}.jpg")
    # Save grayscale image
    cv2.imwrite(filename, current_frame)
    saved_images.append(filename)
    rospy.loginfo(f"Saved image locally: {filename}")

def upload_to_google_drive(file_path):
    try:
        file_metadata = {
            'name': os.path.basename(file_path),
            'parents': [folder_id]
        }
        media = MediaFileUpload(file_path, mimetype='image/jpeg')
        file = service.files().create(body=file_metadata, media_body=media, fields='id').execute()
        rospy.loginfo(f"Uploaded {file_path} to Drive (ID: {file['id']})")
        # Optionally delete:
        # os.remove(file_path)
    except Exception as e:
        rospy.logerr(f"Failed to upload {file_path}: {e}")

def upload_all_images_on_shutdown():
    global recording_active
    recording_active = False
    rospy.loginfo("Uploading all saved images to Google Drive...")
    for path in saved_images:
        upload_to_google_drive(path)
        time.sleep(1.5)
    rospy.loginfo("All session images uploaded.")

# === ROS Callbacks ===

def cmd_vel_callback(msg):
    global current_encoding
    if not recording_active or rospy.is_shutdown():
        return
    if msg.angular.z > 0:
        current_encoding = [1, 0, 0] #left
    elif msg.angular.z < 0:
        current_encoding = [0, 0, 1] #right
    else:
        current_encoding = [0, 1, 0] #forward

last_saved_time = 0  # Global timestamp
target_fps = 30

def rosImage_2_OpenCV(msg):
    global current_frame, last_saved_time, target_fps

    if rospy.is_shutdown() or not recording_active:
        return

    now = time.time()
    if now - last_saved_time < 1/target_fps:  # 5 FPS = save every 0.2 seconds
        return

    try:
        # Convert ROS Image message to OpenCV BGR image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Resize to 128x128
        resized_image = cv2.resize(gray_image, (128, 128))

        current_frame = resized_image
        last_saved_time = now

        save_image_with_one_hot_encoding(current_encoding)

    except CvBridgeError as e:
        rospy.logerr(f"Image conversion failed: {e}")

# === Main ROS Node ===

def track_teleop_commands():
    rospy.init_node('teleop_cmd_tracker', anonymous=True)
    rospy.loginfo(f"Started image tracker (PID {os.getpid()})")
    rospy.on_shutdown(upload_all_images_on_shutdown)

    rospy.Subscriber('/B1/cmd_vel', Twist, cmd_vel_callback)
    rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, rosImage_2_OpenCV)

    rospy.spin()

if __name__ == '__main__':
    try:
        track_teleop_commands()
    except rospy.ROSInterruptException:
        pass


#!/usr/bin/env python3
import rospy
import torch
import torch.nn as nn
import torchvision.transforms as transforms
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time  # add at the top

# === CNN model for grayscale 128x128 input ===
class CNNModel(nn.Module):
    def __init__(self):
        super(CNNModel, self).__init__()
        self.net = nn.Sequential(
            nn.Conv2d(1, 16, 5, 2, 2),  # 1 input channel for grayscale
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 5, 2, 2),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, 2, 1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1)),
            nn.Flatten(),
            nn.Linear(64, 3)
        )

    def forward(self, x):
        return self.net(x)

class InferenceNode:
    def __init__(self):
        rospy.init_node("imitation_inference_node")

        # === Load model ===
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = CNNModel().to(self.device)

        model_path = "/home/fizzer/ros_ws/src/Controller_pkg/scripts/Model_V2.pth"
        if torch.cuda.is_available():
            self.model.load_state_dict(torch.load(model_path))
        else:
            self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()

        # === ROS setup ===
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.callback)
        self.pub = rospy.Publisher("/B1/cmd_vel", Twist, queue_size=10)

        # === Image transform (for grayscale 128x128) ===
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Grayscale(num_output_channels=1),
            transforms.Resize((128, 128)),
            transforms.ToTensor(),
        ])

        # === Frame skipping control ===
        self.target_fps = 30  # process only 5 frames per second
        self.last_processed_time = 0.0

    def callback(self, msg):
        current_time = time.time()

        # Only process if enough time has passed
        if current_time - self.last_processed_time < 1.0 / self.target_fps:
            return  # Skip this frame

        self.last_processed_time = current_time

        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # If already grayscale (single channel), no need to convert
            if len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Apply transform and prepare tensor
            input_tensor = self.transform(cv_image).unsqueeze(0).to(self.device)

            # Run inference
            with torch.no_grad():
                output = self.model(input_tensor)
                prediction = torch.argmax(output, dim=1).item()

            # Map prediction to Twist command
            twist = Twist()
            if prediction == 0:  # Left
                twist.angular.z = 2.143588
                twist.linear.x = 1.07179
            elif prediction == 1:  # Forward
                twist.linear.x = 1.07179
            elif prediction == 2:  # Right
                twist.angular.z = -2.143588
                twist.linear.x = 1.07179

            self.pub.publish(twist)

        except Exception as e:
            rospy.logerr(f"Failed to process image: {e}")


    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = InferenceNode()
    node.run()

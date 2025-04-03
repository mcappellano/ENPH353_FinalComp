#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf

class VelocityPredictorNode:
    def __init__(self):
        rospy.init_node("velocity_predictor_node")

        # Load model
        model_path = "/home/fizzer/ros_ws/src/Controller_pkg/scripts/velocity_predictor_model.h5"
        self.model = tf.keras.models.load_model(model_path)
        rospy.loginfo(f"✅ Loaded model from {model_path}")

        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/B1/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.image_callback)

        # Image preprocessor
        self.input_size = (128, 128)

    def preprocess_image(self, cv_image):
        """Resize, normalize, and add batch/channel dimensions."""
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        resized = cv2.resize(gray, self.input_size)
        normalized = resized.astype(np.float32) / 255.0
        expanded = np.expand_dims(normalized, axis=(0, -1))  # Shape: (1, 128, 128, 1)
        return expanded

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            input_tensor = self.preprocess_image(cv_image)

            prediction = self.model.predict(input_tensor)[0]
            linear, angular = prediction[0], prediction[1]

            # Build and publish Twist message
            twist = Twist()
            twist.linear.x = linear * 0.5  # reduce forward speed
            twist.angular.z = angular * 0.7  # smooth turning
            self.pub.publish(twist)

            rospy.loginfo(f"🚗 Predicted lin: {linear:.2f}, ang: {angular:.2f}")

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = VelocityPredictorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

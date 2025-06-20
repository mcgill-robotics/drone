import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
import numpy as np
import os


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('dummy_computer_vision')

        self.cap = cv2.VideoCapture(
            "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1920,height=1080,framerate=60/1 ! nvvidconv ! appsink",
            cv2.CAP_GSTREAMER)

        if (self.cap.isOpened()):
            print("The camera is successfully opened")
        else:
            print("Could not open the camera")
            exit()

        # Camera feed
        self.image_publisher = self.create_publisher(Image, "/video_feed", 10)
        self.bridge = CvBridge()
        timer_period2 = 0.1
        self.timer = self.create_timer(timer_period2, self.passive_feed_pub)

    def publish_frame(self, frame):
        frame = cv2.resize(frame, (1280, 720))
        # publishes message
        self.image_publisher.publish(
            self.bridge.cv2_to_imgmsg(frame, encoding="passthrough"))

    def passive_feed_pub(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if (not ret):
                print("Unable to get camera frame")
                return
            self.publish_frame(frame)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

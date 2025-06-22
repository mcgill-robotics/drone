import rclpy
import socket
from rclpy.node import Node

import cv2
import numpy as np
import os

TCP_IP = "192.168.144.134"
# TCP_IP = "localhost"
TCP_PORT = 8080


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
        # self.image_publisher = self.create_publisher(Image, "/video_feed", 10)
        # self.bridge = CvBridge()
        timer_period2 = 10.
        self.socket = socket.socket()
        self.socket.connect((TCP_IP, TCP_PORT))
        self.timer = self.create_timer(timer_period2, self.passive_feed_pub)

    def publish_frame(self, frame):
        #     # publishes message
        #     self.image_publisher.publish(
        #         self.bridge.cv2_to_imgmsg(frame, encoding="passthrough"))
        # frame = cv2.resize(frame, (1280, 720))
        # cv2.imshow("Original", frame)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        res, imencode = cv2.imencode(".jpg", frame, encode_param)
        data = np.array(imencode)
        stringData = data.tostring()
        self.socket.send(str(len(stringData)).ljust(16).encode())
        self.socket.send(stringData)

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

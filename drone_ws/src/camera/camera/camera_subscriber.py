import rclpy
# from cv_bridge import CvBridge
from rclpy.node import Node
# from sensor_msgs.msg import Image
import socket

import cv2
import numpy as np
import os
from pathlib import Path

TCP_IP = "0.0.0.0"
TCP_PORT = 8080
images_path = Path.home() / "mapping" / "images"


class MinimalPublisher(Node):

    @staticmethod
    def recvall(sock, count):
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf:
                return None
            buf += newbuf
            count -= len(newbuf)
        return buf

    def __init__(self):
        super().__init__('dummy_computer_vision')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((TCP_IP, TCP_PORT))
        self.socket.listen(True)
        index = 0
        while (True):
            conn, addr = self.socket.accept()
            length = int(self.recvall(conn, 16))
            stringData = self.recvall(conn, length)
            data = np.fromstring(stringData, dtype="uint8")

            img = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
            cv2.imwrite((images_path / f"image{index}.jpg").resolve(), img)
            index += 1


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

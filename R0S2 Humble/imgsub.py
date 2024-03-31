#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)
        
        self.cv_bridge = CvBridge()

    def listener_callback(self, data):
        self.get_logger().info('Receiving image')
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.imshow("Camera Feed", image)
        
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        sobel_x = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=5)
        sobel_y = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=5)
        sobel_combined = cv2.addWeighted(cv2.convertScaleAbs(sobel_x), 0.5, cv2.convertScaleAbs(sobel_y), 0.5, 0)
        edges = cv2.Canny(sobel_combined, 500, 600)
        hist = cv2.calcHist([sobel_combined], [0], None, [256], [0,256])

        cv2.imshow("Sobel Filtered Image", sobel_combined)
        cv2.imshow('Canny Edge Detection', edges)
        cv2.imshow('Histogram', hist)
        cv2.waitKey(10)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber("imgsub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
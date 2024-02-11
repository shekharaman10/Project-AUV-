#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from functools import partial


class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub_= self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Draw circle node has been started")
        self.quarter = 0
    
    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_pub_.publish(msg)
        self.set_pen_color()

    def set_pen_color(self):
        if self.quarter == 0:
            self.call_set_pen_service(255, 0, 0, 3, 0)  # Red
        elif self.quarter == 1:
            self.call_set_pen_service(0, 255, 0, 3, 0)  # Green
        elif self.quarter == 2:
            self.call_set_pen_service(0, 0, 255, 3, 0)  # Blue
        elif self.quarter == 3:
            self.call_set_pen_service(255, 255, 0, 3, 0)  # Yellow
        self.quarter = (self.quarter + 1) % 4

        #multi-color trail of circle


    def call_set_pen_service(self, r, g, b,width,off):
        client = self.create_client(SetPen, "/turtle1/set_pen")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for service..")

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))
    
    def callback_set_pen(self, future):
        try:
            response = future.result()
            self.get_logger().info("Set pen service call successful")
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


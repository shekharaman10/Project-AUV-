#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from functools import partial

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__('turtle_controller')

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.get_logger().info('Turtle controller has been started')

    def pose_callback(self, pose):
        cmd = Twist()
        cmd.linear.x = 2.0
        cmd.angular.z = 1.0

        # Set pen colors based on turtle's coordinates
        if pose.x > 5.54 and pose.y > 5.54:
            self.call_set_pen_service(255, 0, 0, 3)  # Red
        elif pose.x < 5.54 and pose.y > 5.54:
            self.call_set_pen_service(0, 255, 0, 3)  # Green
        elif pose.x < 5.54 and pose.y < 5.54:
            self.call_set_pen_service(100, 155, 0, 3)  # Orange
        elif pose.x > 5.54 and pose.y < 5.54:
            self.call_set_pen_service(0, 255, 255, 3)  # Cyan

        # Publish the twist command
        self.cmd_vel_pub.publish(cmd)

    def call_set_pen_service(self, r, g, b, width):
        client = self.create_client(SetPen, '/turtle1/set_pen')
        if not client.service_is_ready():
            self.get_logger().warn('Service not available')
            return

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))

    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


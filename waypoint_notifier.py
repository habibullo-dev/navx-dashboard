import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


def distance(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


class WaypointNotifier(Node):
    def __init__(self):
        super().__init__('waypoint_notifier')

        # Subscribe to odom
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Services to save current pose as home/destination
        self.srv_home = self.create_service(Empty, 'set_home', self.set_home_callback)
        self.srv_dest = self.create_service(Empty, 'set_destination', self.set_destination_callback)

        # Stored waypoints: (x, y) or None
        self.home = None
        self.destination = None

        # Flags so we don't spam logs every callback
        self.home_reached = False
        self.dest_reached = False

        # Distance threshold in meters
        self.reach_threshold = 0.20  # 20 cm

        self.current_pose = (0.0, 0.0)
        self.get_logger().info("WaypointNotifier node started.")

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_pose = (x, y)

        # Check home
        if self.home is not None:
            d_home = distance(self.current_pose, self.home)
            if d_home <= self.reach_threshold and not self.home_reached:
                self.home_reached = True
                self.get_logger().info("✅ Home reached! (distance: %.3f m)" % d_home)
            elif d_home > self.reach_threshold:
                # If we move away again, allow future notifications
                self.home_reached = False

        # Check destination
        if self.destination is not None:
            d_dest = distance(self.current_pose, self.destination)
            if d_dest <= self.reach_threshold and not self.dest_reached:
                self.dest_reached = True
                self.get_logger().info("🎯 Destination reached! (distance: %.3f m)" % d_dest)
            elif d_dest > self.reach_threshold:
                self.dest_reached = False

    def set_home_callback(self, request, response):
        self.home = self.current_pose
        self.home_reached = False
        self.get_logger().info("🏠 Home saved at x=%.3f, y=%.3f" % self.home)
        return response

    def set_destination_callback(self, request, response):
        self.destination = self.current_pose
        self.dest_reached = False
        self.get_logger().info("📍 Destination saved at x=%.3f, y=%.3f" % self.destination)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNotifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

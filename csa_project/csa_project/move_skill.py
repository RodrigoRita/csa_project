#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Twist
from rclpy.node import Node


class Move_skill(Node):
    def __init__(self, namespace):
        # Use the namespace in the Node constructor
        node_name = "move_skill"
        super().__init__(node_name)

        # Robot state (from AMCL)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Active goal
        self.goal = None
        self.goal_reached = False

        # Topics with namespace
        cmd_topic = "cmd_vel"
        goal_topic = "goal"
        amcl_topic = "amcl_pose"  # if AMCL is namespaced

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        # Subscribers
        self.create_subscription(
            PoseWithCovarianceStamped, amcl_topic, self.amcl_callback, 10
        )

        self.create_subscription(Point, goal_topic, self.goal_callback, 10)

        # Control loop (10 Hz)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f"Waiting for {goal_topic} and {amcl_topic}...")

    def goal_callback(self, msg: Point):
        """Store new goal from /goal."""
        self.goal = (msg.x, msg.y)
        self.goal_reached = False
        self.get_logger().info(f"New goal: ({msg.x:.2f}, {msg.y:.2f})")

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """Update robot pose using AMCL (not odom!)."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def control_loop(self):
        if self.goal is None or self.goal_reached:
            return

        gx, gy = self.goal
        dx = gx - self.x
        dy = gy - self.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < 0.10:
            self.stop()
            self.goal_reached = True
            self.get_logger().info("Reached goal.")
            return

        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.yaw)

        cmd = Twist()
        k_linear = 0.8
        k_angular = 2.0
        cmd.angular.z = k_angular * yaw_error

        if abs(yaw_error) < 0.3:
            cmd.linear.x = k_linear * distance
        else:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    # Example: create GoToXY node for AGV2
    node = Move_skill(agv_namespace="AGV1")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

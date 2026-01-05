#!/usr/bin/env python3
import math
from enum import Enum
from typing import Optional, Tuple

import rclpy
from csa_helper.srv import RequestTransport
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class MoveState(Enum):
    ALIGN = 1
    DRIVE = 2


class TransportAgent(Node):
    """
    TransportAgent node with internal move skill.
    """

    def __init__(self):
        super().__init__("transport_agent")
        self.agv_name = self.get_name()

        # ----- internal state -----
        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = 0.0

        self.goal: Optional[Tuple[float, float]] = None
        self.goal_reached: bool = False
        self.occupied: bool = False  # True while moving / doing transport
        self.amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )
        self.state = MoveState.ALIGN
        self.goal_yaw = None  # optional final yaw

        # ----- Publishers -----
        self.cmd_pub = self.create_publisher(Twist, f"/{self.agv_name}/cmd_vel", 10)
        self.occupied_pub = self.create_publisher(
            Bool, f"/{self.agv_name}/occupied_transport", 10
        )
        self.goal_reached_pub = self.create_publisher(
            Bool, f"/{self.agv_name}/goal_reached", 10
        )

        # ----- Subscribers -----
        self.create_subscription(
            Odometry,
            f"/{self.agv_name}/odom",
            self._odom_callback,
            10,
        )

        # ----- Service -----
        self.request_transport_srv = self.create_service(
            RequestTransport,
            f"/{self.agv_name}/request_transport",
            self._handle_request_transport,
        )
        self.shutdown_srv = self.create_service(
            Trigger, f"{self.agv_name}/shutdown", self.shutdown_callback
        )

        # ----- Timers -----
        self.create_timer(0.1, self._control_loop)  # control loop 10 Hz
        self.create_timer(0.2, self._publish_occupied)  # occupancy 5 Hz

        self.get_logger().info(f"TransportAgent [{self.agv_name}] started.")

    # -------------------------
    # Callbacks / helpers
    # -------------------------
    def _odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def _publish_occupied(self):
        """Publish whether AGV is occupied to ResourceController."""
        m = Bool()
        m.data = self.occupied
        self.occupied_pub.publish(m)

    def _handle_request_transport(self, request, response):
        """Service handler for RequestTransport."""
        self.get_logger().info(
            f"[TA] Transport requested by {request.product_type}{request.id}"
        )
        if self.occupied:
            self.get_logger().warn(
                f"{self.agv_name}: received request but currently occupied."
            )
            response.task_accepted = False
            return response

        dest: Pose2D = request.destination
        self.goal = (dest.x, dest.y)
        self.goal_reached = False
        self.occupied = True
        self.state = MoveState.ALIGN

        self.get_logger().info(
            f"{self.agv_name}: accepted transport request → goal=({dest.x:.2f}, {dest.y:.2f})"
        )

        response.task_accepted = True
        return response

    # ------------------------------------------------------------------
    # Shutdown service
    # ------------------------------------------------------------------
    def shutdown_callback(self, request, response):
        if self.occupied:
            response.success = False
            response.message = f"{self.agv_name} busy, shutdown refused."
            return response

        response.success = True
        response.message = f"{self.agv_name} shutting down"
        rclpy.shutdown()
        return response

    def _control_loop(self):
        if self.goal is None:
            return

        gx, gy = self.goal
        dx = gx - self.x
        dy = gy - self.y
        distance = math.hypot(dx, dy)

        target_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(target_yaw - self.yaw)

        cmd = Twist()

        # ------------------------
        # STATE 1: ALIGN TO GOAL
        # ------------------------
        if self.state == MoveState.ALIGN:
            if abs(yaw_error) > 0.05:
                cmd.angular.z = max(min(2.0 * yaw_error, 1.5), -1.5)
                cmd.linear.x = 0.0
            else:
                self.state = MoveState.DRIVE

        # ------------------------
        # STATE 2: DRIVE STRAIGHT
        # ------------------------
        elif self.state == MoveState.DRIVE:
            if distance > 0.15:
                k_linear = 2.0
                k_angular = 2.0

                cmd.angular.z = max(min(k_angular * yaw_error, 1.0), -1.0)

                heading_scale = max(0.5, 1.0 - 0.5 * abs(yaw_error))
                cmd.linear.x = min(k_linear * distance, 2.5) * heading_scale
            else:
                self._stop_robot()
                self.goal_reached = True
                self.occupied = False
                self.goal = None
                self.state = MoveState.ALIGN

                self.goal_reached_pub.publish(Bool(data=True))
                self.get_logger().info(f"{self.agv_name}: goal reached")
                return

        self.cmd_pub.publish(cmd)

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _normalize_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = TransportAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

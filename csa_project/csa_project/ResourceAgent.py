#!/usr/bin/env python3
import rclpy
from csa_helper.action import ExecuteSkill
from csa_helper.msg import Station
from csa_helper.srv import GetResources, ReserveStation, StartSkills
from geometry_msgs.msg import Pose2D
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, ColorRGBA
from std_srvs.srv import Trigger


class ResourceAgent(Node):
    """
    ResourceAgent:
    - Provides `{resource_name}/get_resources` service for ResourceController
    - Provides `{resource_name}/start_skills` service for ProductAgent
    - Executes skills via ExecuteSkill Action
    """

    def __init__(self):
        super().__init__(
            "resource_agent"
        )  # Name does not matter as it is overritten when node is created in ResourceController

        # --------------------------
        # Parameters
        # --------------------------
        self.declare_parameter("associatedSkills", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("location", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("skillsTime", Parameter.Type.DOUBLE_ARRAY)

        self.skills = {}

        self.associatedSkills = (
            self.get_parameter("associatedSkills")
            .get_parameter_value()
            .string_array_value
        )
        self.skillsTime = (
            self.get_parameter("skillsTime").get_parameter_value().double_array_value
        )
        for i in range(len(self.skillsTime)):
            self.skills[self.associatedSkills[i]] = self.skillsTime[i]

        loc = self.get_parameter("location").value
        self.location = Pose2D(x=loc[0], y=loc[1], theta=loc[2])

        self.resource_name = self.get_name()
        self.occupied = False
        self.operating = False  # skill currently executing

        # --------------------------
        # Publishers
        # --------------------------
        self.light_pub = self.create_publisher(
            ColorRGBA, f"/{self.resource_name}/light_color", 10
        )
        self.create_timer(1.0, self.light_callback)

        self.skill_finished_pub = self.create_publisher(
            Bool, f"/{self.resource_name}/skillFinished", 10
        )

        # --------------------------
        # Services
        # --------------------------
        self.get_resources_srv = self.create_service(
            GetResources,
            f"{self.resource_name}/get_resources",
            self.handle_resource_request,
        )

        self.start_skills_srv = self.create_service(
            StartSkills,
            f"{self.resource_name}/start_skills",
            self.start_skills_callback,
        )

        self.reserve_station_srv = self.create_service(
            ReserveStation,
            f"{self.resource_name}/reserve_station",
            self.reserve_station_callback,
        )
        self.shutdown_srv = self.create_service(
            Trigger, f"{self.resource_name}/shutdown", self.shutdown_callback
        )
        # --------------------------
        # Action Client
        # --------------------------
        self.skill_client = ActionClient(self, ExecuteSkill, "execute_skill")

        self.get_logger().info(
            f"ResourceAgent [{self.resource_name}] ready. Skills: {self.associatedSkills}"
        )

    # ------------------------------------------------------------------
    # Light visualization
    # ------------------------------------------------------------------
    def light_callback(self):
        msg = ColorRGBA()
        if self.occupied and not self.operating:
            msg.r, msg.g, msg.b, msg.a = 1.0, 1.0, 0.0, 1.0
        if self.operating:
            msg.r, msg.g, msg.b, msg.a = 1.0, 0.0, 0.0, 1.0
        else:
            msg.r, msg.g, msg.b, msg.a = 0.0, 1.0, 0.0, 1.0
        self.light_pub.publish(msg)

    # ------------------------------------------------------------------
    # get_resources service
    # ------------------------------------------------------------------
    def handle_resource_request(self, request, response):
        response.station = Station()
        response.station.name = ""
        response.station.occupied = self.occupied
        response.station.location = self.location

        if request.current_task not in self.associatedSkills:
            return response

        response.station.name = self.resource_name
        return response

    def reserve_station_callback(self, request, response):
        if request.occupied:
            response.ack = True
            self.occupied = True
            return response

    # ------------------------------------------------------------------
    # start_skills service
    # ------------------------------------------------------------------
    def start_skills_callback(self, request, response):
        # Check if action is online
        if not self.skill_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn("ExecuteSkill server not available yet")
            response.task_accepted = False
            return response

        if self.operating:
            response.task_accepted = False
            return response

        self.get_logger().info(
            f"[RA] Starting skill '{request.current_task}' on {self.resource_name}"
        )

        goal = ExecuteSkill.Goal()
        goal.task = request.current_task
        goal.station = self.resource_name
        goal.skill_time = self.skills[request.current_task]
        goal.agv = request.agv_name
        goal.color = request.color

        send_goal_future = self.skill_client.send_goal_async(
            goal,
            feedback_callback=self.skill_feedback_callback,
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        response.task_accepted = True
        self.operating = True
        return response

    # ------------------------------------------------------------------
    # Shutdown service
    # ------------------------------------------------------------------
    def shutdown_callback(self, request, response):
        if self.operating or self.occupied:
            response.success = False
            response.message = f"{self.resource_name} busy, shutdown refused."
            return response

        response.success = True
        response.message = f"{self.resource_name} shutting down"
        rclpy.shutdown()
        return response

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Skill goal rejected")
            self.occupied = False
            self.operating = False
            return

        self.get_logger().info("Skill goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.skill_result_callback)

    def skill_feedback_callback(self, feedback_msg):
        self.get_logger().info(f"[Skill feedback] {feedback_msg.feedback.state}")

    def skill_result_callback(self, future):
        result = future.result().result

        if result.success:
            self.get_logger().info("Skill completed successfully")

        else:
            self.get_logger().error(f"Skill failed: {result.message}")

        self.occupied = False
        self.operating = False
        msg = Bool()
        msg.data = result.success
        self.skill_finished_pub.publish(msg)
        self.get_logger().info("Skill finished → notifying ProductAgent")


def main(args=None):
    rclpy.init(args=args)
    node = ResourceAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

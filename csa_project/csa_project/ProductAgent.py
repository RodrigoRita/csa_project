#!/usr/bin/env python3
from enum import Enum

import rclpy
from csa_helper.msg import ControllerResponse
from csa_helper.srv import (
    ProductResource,
    RequestTransport,
    SetProductColor,
    StartSkills,
)
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, ColorRGBA


class SubGoal(Enum):
    NONE = 0
    COMPLETED = 1
    PRESENT = 2


class ProductAgent(Node):
    """Non-blocking ProductAgent using asynchronous callbacks."""

    def __init__(self, product_id):
        super().__init__(f"product_{product_id}")

        # ---------------- Parameters ----------------
        self.declare_parameter("product_id", product_id)
        self.declare_parameter("product_type", "")
        self.declare_parameter("execution_plan", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("color", Parameter.Type.DOUBLE_ARRAY)

        self.id = self.get_parameter("product_id").value
        self.type = self.get_parameter("product_type").value
        self.execution_plan = (
            self.get_parameter("execution_plan")
            .get_parameter_value()
            .string_array_value
        )
        self.product_name = self.get_name()
        self.get_logger().info(self.product_name)

        color_list = (
            self.get_parameter("color").get_parameter_value().double_array_value
        )
        self.product_color = ColorRGBA(
            r=color_list[0],
            g=color_list[1],
            b=color_list[2],
            a=color_list[3],
        )

        # State machine
        self.current_task_index = 0
        self.current_task = ""
        self.current_agv = ""
        self.current_station = ""
        self.current_location_str = ""  # To send to ResourceController
        self.current_location = Pose2D()  # To send to TransportAgent
        self.current_destination = Pose2D()

        # Flags
        self.resources_found = (
            False  # True if the response from request to RC is sucessfull
        )
        self.retry_skill_timer = None
        self.retry_task_timer = None
        self.nextProductLaunched = False
        self.sub_goal = SubGoal.NONE  # Is there a sub_goal needed to be achieved?
        self.transport_goal_reached = False
        self.transport_sub_goal_reached = False
        self.skill_finished = False
        self.waiting_for_resources = False
        self.resources_ready = False

        # Publishers
        self.next_product_pub = self.create_publisher(Bool, "/next_product", 10)

        # Subscribers (they will be initialized dynamically)
        self.goal_subs = {}
        self.skill_subs = {}

        self.create_subscription(
            ControllerResponse,
            "/controller_response",
            self.controller_response_callback,
            10,
        )

        # Resource controller client
        self.rc_client = self.create_client(ProductResource, "/product_resource")
        while not self.rc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /product_resource service...")

        # Transport Clients
        self.transport_clients = {}
        self.station_clients = {}
        self.station_clients_reserve = {}

        # Set Product Color Client
        self.product_loc_cli = self.create_client(SetProductColor, "set_product_body")
        while not self.product_loc_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        # Start execution
        self.execute_next_task()

    # ----------------------------- STATE MACHINE -----------------------------

    def execute_next_task(self):
        """Start the next task from the execution plan."""
        if self.current_task_index >= len(self.execution_plan):
            self.get_logger().info("Product execution completed.")
            return

        self.current_task = self.execution_plan[self.current_task_index]
        self.get_logger().info(f"[PA] Starting task: {self.current_task}")

        # Reset flags
        self.resources_found = False
        self.transport_sub_goal_reached = False
        self.transport_goal_reached = False
        self.skill_finished = False

        # Ask ResourceController for resources
        req = ProductResource.Request()
        req.current_task = self.current_task
        req.product_type = self.type
        req.id = self.id
        req.location_str = self.current_location_str

        self.rc_client.call_async(req)

    # ---------------------------- CALLBACKS ---------------------------------
    def controller_response_callback(self, msg: ControllerResponse):
        # Ignore messages not meant for this product
        if msg.product_name != self.get_name():
            return
        if not msg.station.name or not msg.agv.name:
            self.resources_found = False
            self.retry_request_task()  # No Resources available? Retry request
            return

        if msg.sub_goal:
            self.sub_goal = SubGoal.PRESENT
        else:
            self.sub_goal = SubGoal.NONE

        self.current_station = msg.station.name
        self.current_destination = msg.station.location
        self.current_agv = msg.agv.name
        self.current_location = msg.location
        self.resources_found = True
        self.get_logger().info(
            f"[PA] RC decided: Station={self.current_station}, AGV={self.current_agv}"
        )

        # If first task is pickUp → allow next product
        if not self.nextProductLaunched and not self.current_task_index == 0:
            self.nextProductLaunched = True
            trigger = Bool()
            trigger.data = True
            self.next_product_pub.publish(trigger)

        # Continue FSM
        if self.current_agv:
            self.start_transport_phase()
        else:
            self.start_skill_phase()

    # ---------------------------- TRANSPORT PHASE ----------------------------

    def start_transport_phase(self):
        agv = self.current_agv
        self.get_logger().info(f"[PA] Requesting transport by {agv}")

        if agv not in self.transport_clients:
            self.transport_clients[agv] = self.create_client(
                RequestTransport, f"/{agv}/request_transport"
            )

        client = self.transport_clients[agv]

        req = RequestTransport.Request()
        req.current_task = self.current_task
        req.product_type = self.type
        req.id = self.id
        if self.sub_goal == SubGoal.NONE:

            # Before moving, transfer the product from the station to the AGV
            agv_product_color_update = SetProductColor.Request()
            agv_product_color_update.resource_name = self.current_agv
            agv_product_color_update.color = self.product_color
            agv_product_color_update.enable = True
            self.product_loc_cli.call_async(agv_product_color_update)

            station_product_color_update = SetProductColor.Request()
            station_product_color_update.resource_name = self.current_location_str
            station_product_color_update.color = self.product_color
            station_product_color_update.enable = False
            self.product_loc_cli.call_async(station_product_color_update)

            req.destination = self.current_destination  # Move to goal
        elif self.sub_goal == SubGoal.PRESENT:
            req.destination = self.current_location  # Move to sub_goal
        else:
            req.destination = self.current_destination  # Moe to final_goal
        # Create subscribe to AGV goal_reached
        if agv not in self.goal_subs:
            self.goal_subs[agv] = self.create_subscription(
                Bool,
                f"/{agv}/goal_reached",
                self.goal_reached_callback,
                10,
            )
        future = client.call_async(req)
        future.add_done_callback(self.on_transport_accepted)

    def on_transport_accepted(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Transport request failed: {e}")
            self.retry_request_skill()
            return

        if not res.task_accepted:
            self.get_logger().warn("Transport task was rejected.")
            self.retry_request_skill()
            return

        self.get_logger().info(f"[PA] AGV accepted. Waiting for goal...")

    def goal_reached_callback(self, msg):
        if not msg.data:
            return

        # Ignore duplicate callbacks
        if self.transport_goal_reached:
            self.get_logger().warn("[PA] Duplicate goal_reached ignored")
            return

        if self.sub_goal == SubGoal.PRESENT:
            self.sub_goal = SubGoal.COMPLETED  # Sub goal completed

            # Adicionar lógica de mudança de cor para simular retirada do produto da estação. Usar location_str
            agv_product_color_update = SetProductColor.Request()
            agv_product_color_update.resource_name = self.current_agv
            agv_product_color_update.color = self.product_color
            agv_product_color_update.enable = True
            self.product_loc_cli.call_async(agv_product_color_update)
            if self.current_location_str:
                station_product_color_update = SetProductColor.Request()
                station_product_color_update.resource_name = self.current_location_str
                station_product_color_update.color = self.product_color
                station_product_color_update.enable = False
                self.product_loc_cli.call_async(station_product_color_update)

            self.get_logger().info("[PA] Transport sub_goal reached.")
            self.transport_sub_goal_reached = True
            self.start_transport_phase()
        else:
            self.get_logger().info("[PA] Transport goal reached.")
            self.transport_goal_reached = True
            self.start_skill_phase()

    # ----------------------------- SKILL PHASE -------------------------------

    def start_skill_phase(self):
        """Send start_skill request once AGV has delivered the product."""
        if not self.transport_goal_reached:
            self.get_logger().info("[PA] Waiting for transport to finish.")
            return

        station = self.current_station
        agv = self.current_agv

        if station not in self.station_clients:
            self.station_clients[station] = self.create_client(
                StartSkills, f"/{station}/start_skills"
            )
        client = self.station_clients[station]

        req = StartSkills.Request()
        req.current_task = self.current_task
        req.product_type = self.type
        req.color = self.product_color
        req.id = self.id
        req.station_name = station
        req.agv_name = agv

        future = client.call_async(req)
        future.add_done_callback(self.on_skill_accepted)

        if station not in self.skill_subs:
            self.skill_subs[station] = self.create_subscription(
                Bool,
                f"/{station}/skillFinished",
                self.skill_finished_callback,
                10,
            )

    def on_skill_accepted(
        self, future
    ):  # TODO: Colocar a lógica de mudança de cor para o produto da station (COR) e AGV (NADA)
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Skill request failed: {e}")
            self.retry_request_skill()
            return

        if not res.task_accepted:
            self.get_logger().warn("Skill was rejected.")
            self.retry_request_skill()
            return

        # Task accepted, product being processed, simulate item transferance
        # --- pickUP (Change AGV's product body to color)
        if self.current_task == "pickUp":
            agv_product_color_update = SetProductColor.Request()
            agv_product_color_update.resource_name = self.current_agv
            agv_product_color_update.color = self.product_color
            agv_product_color_update.enable = True
            self.product_loc_cli.call_async(agv_product_color_update)

        # --- drop (Change AGV's product body to transparent)
        else:
            agv_product_color_update = SetProductColor.Request()
            agv_product_color_update.resource_name = self.current_agv
            agv_product_color_update.color = self.product_color
            agv_product_color_update.enable = False
            self.product_loc_cli.call_async(agv_product_color_update)

            if self.current_task != "drop":  # --- Add product to Station
                station_product_color_update = SetProductColor.Request()
                station_product_color_update.resource_name = self.current_station
                station_product_color_update.color = self.product_color
                station_product_color_update.enable = True
                self.product_loc_cli.call_async(station_product_color_update)

        self.get_logger().info("[PA] Skill accepted. Waiting for completion...")

    def skill_finished_callback(self, msg):
        if not msg.data:
            return
        if self.skill_finished or not self.transport_goal_reached:
            self.get_logger().warn("[PA] Duplicate skillFinished ignored")
            return

        self.skill_finished = True
        self.current_location_str = self.current_station  # Update product localization

        # Move to next task
        self.current_task_index += 1
        self.execute_next_task()

    def retry_request_skill(self, delay=1.0):
        if self.retry_skill_timer:
            return  # already scheduled

        self.get_logger().info("[PA] Scheduling resource retry...")
        self.retry_skill_timer = self.create_timer(delay, self._retry_callback)

    def retry_request_task(self, delay=4.0):
        if self.retry_task_timer:
            return  # already scheduled

        self.get_logger().info("[PA] Scheduling resource retry...")
        self.retry_task_timer = self.create_timer(delay, self._retry_callback)

    def _retry_callback(self):
        self.retry_task_timer.cancel()
        self.retry_task_timer = None
        # Logic to re-perform the skills
        if (
            not self.resources_found
        ):  # No resources found, restart task completion request
            self.execute_next_task()
        elif not (
            self.transport_goal_reached or self.transport_sub_goal_reached
        ):  # Transport failed, restart transport
            self.start_transport_phase()
        elif (
            not self.skill_finished
        ):  # Skill failed or station temporarily occupied, retry later
            self.start_skill_phase()


# ----------------------------- MAIN -----------------------------------------


def main(args=None):
    rclpy.init(args=args)
    product_agent = ProductAgent(product_id=1)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(product_agent)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        product_agent.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

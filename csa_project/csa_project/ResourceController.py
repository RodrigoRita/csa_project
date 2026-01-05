import math
import subprocess

import rclpy
from csa_helper.msg import Agv, ControllerResponse, Order, Product, Station
from csa_helper.srv import GetResources, ProductResource, ReserveStation
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class ResourceController(Node):
    """Coordinates production by managing resources and AGVs."""

    def __init__(self):
        super().__init__("ResourceController")

        # Product types and their execution plans
        self.current_product_types = {
            "A": ["pickUp", "A", "B", "drop"],
            "B": ["pickUp", "A", "C", "drop"],
        }

        # Current orders and production
        self.current_order = []
        self.current_production = []
        self.productsLocation = {}  # {"A0":{"Station1":Pose2D()}, ...}

        # Current product and task being processed
        self.currentTask = ""
        self.product_name = ""
        self.current_product_localization = ""
        self.sub_goal = False

        # Station and AGV management
        self.station_skills = {}
        self.stationsLocation = {}
        self.skillsTime = {}
        self.AGV_poses = {}
        self.AGVs_status = {}  # agv_name → occupied True/False
        self.stations_shutting_down = (
            set()
        )  # Stations ready to be shut down (Can't shut down a station executing a skill, station breakdown during skill will be considered as broken after the skill)
        self.AGVs_shutting_down = set()  # Same above goes for AGVs
        self.AGVs_battery_level = {}  # Future work

        # Services and clients
        self.product_resource_service = self.create_service(
            ProductResource,
            "/product_resource",
            self.handle_product_request,
        )
        self.get_logger().info("Service /product_resource created")
        self.resource_clients = {}
        self.occupied_stations = {}

        # Publishers
        self.controllerResponse = self.create_publisher(
            ControllerResponse, "/controller_response", 10
        )

        # Subscribers
        self.new_product_subscription = self.create_subscription(
            Product, "/new_product", self.new_product_listener, 10
        )
        self.delete_product_subscription = self.create_subscription(
            String, "/delete_product", self.delete_product_listener, 10
        )
        self.new_station_subscription = self.create_subscription(
            Station, "/new_station", self.new_station_listener, 10
        )
        self.delete_station_subscription = self.create_subscription(
            String, "/delete_station", self.delete_station_listener, 10
        )
        self.new_agv_subscription = self.create_subscription(
            Agv, "/new_agv", self.new_agv_listener, 10
        )
        self.delete_agv_subscription = self.create_subscription(
            String, "/delete_agv", self.delete_agv_listener, 10
        )
        self.new_order_subscription = self.create_subscription(
            Order, "/new_order", self.new_order_listener, 10
        )
        self.amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

    def create_agv_pose_subscription(self, agv_name):
        self.create_subscription(
            PoseWithCovarianceStamped,
            f"/{agv_name}/amcl_pose",
            lambda msg, n=agv_name: self.amcl_callback(msg, n),
            self.amcl_qos,
        )

    # ------------------ Dynamic spawning ------------------

    def spawn_resource_agent(
        self, resource_name, associated_skills, location, skills_time
    ):
        # Convert Pose2D into list of floats (x, y, theta)
        location_list = [location.x, location.y, location.theta]

        skills_str = "[" + ",".join(f'"{s}"' for s in associated_skills) + "]"
        skills_time_str = "[" + ",".join(str(s) for s in skills_time) + "]"
        location_str = "[" + ",".join(str(x) for x in location_list) + "]"

        subprocess.Popen(
            [
                "ros2",
                "run",
                "csa_project",
                "ResourceAgent",
                "--ros-args",
                "-r",
                f"__node:={resource_name}",
                "-p",
                f"associatedSkills:={skills_str}",
                "-p",
                f"skillsTime:={skills_time_str}",
                "-p",
                f"location:={location_str}",
            ]
        )
        self.get_logger().info(f"Spawned ResourceAgent: {resource_name}")

    def spawn_transport_agent(self, agv_name):
        """Spawn a TransportAgent node dynamically using subprocess."""
        subprocess.Popen(
            [
                "ros2",
                "run",
                "csa_project",
                "TransportAgent",
                "--ros-args",
                "-r",
                f"__node:={agv_name}",
            ]
        )
        self.get_logger().info(f"Spawned TransportAgent: {agv_name}")

    # --- Subscribers ---
    def new_product_listener(self, msg):
        if msg is None or msg.name in self.current_product_types:
            return
        self.current_product_types[msg.name] = msg.execution_plan
        self.get_logger().info(f'New product type added: "{msg.name}"')

    def delete_product_listener(self, msg):
        if msg is None:
            return
        self.current_product_types.pop(msg.data, None)
        self.get_logger().info(f'Product "{msg.data}" taken out of production')

    def new_station_listener(self, msg):
        if msg is None:
            return

        resource_name = msg.name
        self.station_skills[resource_name] = msg.associated_skills
        self.stationsLocation[resource_name] = msg.location
        # Spawn the resource agent dynamically
        self.spawn_resource_agent(
            resource_name, msg.associated_skills, msg.location, msg.skills_time
        )
        # Immediately create its client
        self.create_get_resource_client(resource_name)
        self.create_reserve_station_client(resource_name)

    def delete_station_listener(self, msg):
        if msg is None:
            return

        resource_name = msg.data

        if resource_name in self.stations_shutting_down:
            self.get_logger().warn(f"{resource_name} already shutting down")
            return

        self.stations_shutting_down.add(resource_name)

        client = self.create_client(Trigger, f"/{resource_name}/shutdown")

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"{resource_name} shutdown service not available")
            self.stations_shutting_down.remove(resource_name)
            return

        req = Trigger.Request()
        future = client.call_async(req)

        future.add_done_callback(
            lambda fut, name=resource_name: self._shutdown_station_response_cb(
                name, fut
            )
        )

    def _shutdown_station_response_cb(self, name, future):
        self.stations_shutting_down.discard(name)

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Shutdown failed for {name}: {e}")
            return

        if not response.success:
            self.get_logger().warn(response.message)
            return

        self.get_logger().info(response.message)

        # Cleanup after confirmation
        self.resource_clients.pop(name, None)
        self.occupied_stations.pop(name, None)
        self.station_skills.pop(name, None)
        self.stationsLocation.pop(name, None)
        if hasattr(self, "collected_resource_responses"):
            self.collected_resource_responses.pop(name, None)

    # --- Transport Subscribers ---
    def new_agv_listener(self, msg):
        agv_name = msg.name

        # subscribe to occupied_transport for this AGV
        self.subscribe_to_agv_occupied(agv_name)
        # subscribe to amcl_pose for this AGV
        self.create_agv_pose_subscription(agv_name)
        # initialize state
        self.AGVs_status[agv_name] = False
        self.spawn_transport_agent(agv_name)

    def delete_agv_listener(self, msg):
        if msg is None:
            return

        agv_name = msg.data

        if agv_name in self.AGVs_shutting_down:
            self.get_logger().warn(f"{agv_name} already shutting down")
            return

        self.AGVs_shutting_down.add(agv_name)

        client = self.create_client(Trigger, f"/{agv_name}/shutdown")

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"{agv_name} shutdown service not available")
            self.AGVs_shutting_down.remove(agv_name)
            return

        req = Trigger.Request()
        future = client.call_async(req)

        future.add_done_callback(
            lambda fut, name=agv_name: self._shutdown_agv_response_cb(name, fut)
        )

    def _shutdown_agv_response_cb(self, name, future):
        self.agvs_shutting_down.discard(name)

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Shutdown failed for {name}: {e}")
            return

        if not response.success:
            self.get_logger().warn(response.message)
            return

        self.get_logger().info(response.message)

        # Cleanup after confirmation
        self.AGV_poses.pop(name, None)
        self.AGVs_status.pop(name, None)

    # --- Order Subscriber ---
    def new_order_listener(self, msg):
        if msg is None:
            return
        self.current_order = msg.order
        self.get_logger().info(f"New order received: {self.current_order}")

    def subscribe_to_agv_occupied(self, agv_name):
        self.create_subscription(
            Bool,
            f"/{agv_name}/occupied_transport",
            lambda msg, n=agv_name: self.update_agv_occupied(n, msg.data),
            10,
        )

    def update_agv_occupied(self, agv_name, occupied):
        self.AGVs_status[agv_name] = occupied

    def amcl_callback(self, msg: PoseWithCovarianceStamped, agv_name: str):
        """Update the robot pose from AMCL."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.AGV_poses[agv_name] = [x, y]
        self.get_logger().debug(
            f"Pose updated for {agv_name}: {self.AGV_poses[agv_name]}"
        )

        # ------------------ Services & Clients ------------------

    def create_get_resource_client(self, resource_name):
        """
        Lazily create a client for a resource agent for service get_resources.
        Returns an existing client if already connected.
        """
        if resource_name in self.resource_clients:
            return self.resource_clients[resource_name]

        service_name = f"{resource_name}/get_resources"
        client = self.create_client(GetResources, service_name)
        self.resource_clients[resource_name] = client

        self.get_logger().info(f"Waiting for service: {service_name}")
        while rclpy.ok() and not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info(f"Waiting for service: {service_name}")

        self.get_logger().info(f"Connected to service: {service_name}")
        return client

    def create_reserve_station_client(self, resource_name):
        """
        Lazily create a client for a resource agent for servic reserve_station.
        Returns an existing client if already connected.
        """
        if resource_name in self.occupied_stations:
            return self.occupied_stations[resource_name]

        service_name = f"{resource_name}/reserve_station"
        client = self.create_client(ReserveStation, service_name)
        self.occupied_stations[resource_name] = client

        self.get_logger().info(f"Waiting for service: {service_name}")
        while rclpy.ok() and not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info(f"Waiting for service: {service_name}")

        self.get_logger().info(f"Connected to service: {service_name}")
        return client

    # ------------------ Product request handling ------------------
    def on_all_resource_responses_collected(self):
        """Called automatically when all resource agents have responded."""
        stations = list(self.collected_resource_responses.values())
        task = self.pending_request_task

        if not stations:
            self.get_logger().warn(f"[RC] No station can perform task '{task}'")
            return

        self.get_logger().info(f"[RC] Stations capable of task {task}: {stations}")
        ## ---No RL for testing
        # --- Select station (placeholder: take first non-occupied) ---
        free_stations = [
            station
            for station in stations
            if not station.occupied or not station.name in self.stations_shutting_down
        ]
        if not free_stations:
            self.get_logger().warn(
                f"[RC] No free stations available for task {task}. Will retry later."
            )
            self.controller_response.product_name = self.product_name
            self.controller_response.station = Station()  # Empty messages
            self.controller_response.agv.name = ""
            self.controllerResponse.publish(self.controller_response)
        else:
            chosen_station = free_stations[0]
            request = ReserveStation.Request()
            request.occupied = True
            client = self.occupied_stations[chosen_station.name]
            client.call_async(request)

            # --- Select AGV ---
            chosen_agv = self.select_best_agv(chosen_station)

            self.controller_response.product_name = self.product_name
            self.controller_response.sub_goal = self.sub_goal
            self.controller_response.location = self.productsLocation[
                self.product_name
            ][self.current_product_localization]
            self.controller_response.station = chosen_station
            self.controller_response.agv.name = chosen_agv
            self.controllerResponse.publish(self.controller_response)

            self.get_logger().info(
                f"[RC] Final decision for product {self.product_name}: Station={chosen_station.name}, AGV={chosen_agv}"
            )
            ## Using RL

    def resource_response_callback(self, rname, future):
        """Asynchronously collect resource responses from each ResourceAgent."""
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"[RC] Service response from {rname} failed: {e}")
            return

        # Valid response?
        if res and res.station.name != "":
            self.get_logger().info(
                f"[RC] {rname} supports task: {self.pending_request_task}"
            )
            self.collected_resource_responses[rname] = res.station
            self.get_logger().info(
                f"[RC] Updated self.collected_resource_responses with -> {rname} : {self.collected_resource_responses[rname]}"
            )
        else:
            self.get_logger().info(
                f"[RC] {rname} does NOT support task {self.pending_request_task}"
            )

        # Check if all agents responded
        self.responses_received += 1
        if self.responses_received == self.total_expected_responses:
            self.on_all_resource_responses_collected()
            self.get_logger().info("[RC] All resource responses collected.")

    def select_best_agv(self, station):
        """
        Compute distances to the chosen station and select the closest free AGV.
        Assumes self.AGVs_status[agv_name] and self.AGV_poses[agv_name] are kept updated by subscribers.
        """

        free_agvs = [agv for agv, occupied in self.AGVs_status.items() if not occupied]
        self.get_logger().info(f"[RC] Free AGVs: {free_agvs}")
        if not free_agvs or len(free_agvs) == 0:
            self.get_logger().warn("[RC] No free AGVs available!")
            return ""

        # Compute distances
        min_distance = 10**6
        chosen = ""
        if self.currentTask == "pickUp":
            self.productsLocation[self.product_name] = {
                station.name: self.stationsLocation[station.name]
            }
            self.current_product_localization = station.name

        product_location, product_pose = next(
            iter(self.productsLocation[self.product_name].items())
        )

        if (
            "AGV" in product_location
        ):  # If the product is in an AGV, that AGV must be chosen
            chosen = product_location  # Sub_goal

        elif (
            product_location == station.name
        ):  # Current location equals goal only one distance to be evaluated
            for agv in free_agvs:
                agv_pose = self.AGV_poses.get(agv)
                self.get_logger().info(f"[RC] Pose fo {agv}: {agv_pose}.")
                if agv_pose is None:
                    continue

                dx = agv_pose[0] - station.location.x
                dy = agv_pose[1] - station.location.y
                distance = (dx * dx + dy * dy) ** 0.5
                self.get_logger().info(f"[RC] Distance for {agv}: {distance}.")
                # Pick closest AGV
                if distance < min_distance:
                    min_distance = distance
                    chosen = agv
        else:  # Sub_goal needed
            self.sub_goal = True
            for agv in free_agvs:
                agv_pose = self.AGV_poses.get(agv)
                self.get_logger().info(f"[RC] Pose fo {agv}: {agv_pose}.")
                if agv_pose is None:
                    continue

                sub_goal = self.productsLocation[self.product_name][product_location]
                d1x = agv_pose[0] - sub_goal.x
                d1y = agv_pose[1] - sub_goal.y
                d2x = sub_goal.x - station.location.x
                d2y = sub_goal.y - station.location.y
                distance = (d1x * d1x + d1y * d1y) ** 0.5 + (
                    d2x * d2x + d2y * d2y
                ) ** 0.5
                self.get_logger().info(f"[RC] Distance for {agv}: {distance}.")
                # Pick closest AGV
                if distance < min_distance:
                    min_distance = distance
                    chosen = agv

        return chosen

    def handle_product_request(self, request, response):
        """
        Receives a product request and starts asynchronous querying of all ResourceAgents.
        Does NOT block! Results arrive in callbacks.
        """
        current_task = request.current_task
        self.currentTask = current_task  # To use for other functions
        product_type = request.product_type
        product_id = request.id
        self.product_name = f"{product_type}_{product_id}"

        # Update last product location. It only doesn't happen for the pickUp stations (WarehouseInput)
        if request.location_str:
            loc = request.location_str
            self.current_product_localization = loc
            if "AGV" in loc:
                self.productsLocation[self.product_name] = {
                    loc: None
                }  # None for AGV for its position is not used
            elif loc in self.station_skills.keys():
                self.productsLocation[self.product_name] = {
                    loc: self.stationsLocation[loc]
                }

        self.get_logger().info(
            f"[RC] Received product request for task: {current_task}"
        )

        # Prepare the response (will be filled asynchronously)
        response.station = Station()
        response.agv = Agv()
        self.controller_response = ControllerResponse()
        # In this step, it was supposed to choose the resources and then return the response, which it is not possible due to higher proccess time and callbacks
        self.pending_product_response = response
        self.pending_request_task = current_task
        self.pending_request_id = product_id
        self.pending_request_product_type = product_type
        self.responses_received = 0

        # Reset response buffer for this request
        self.collected_resource_responses = {}  # rname → Station
        self.total_expected_responses = len(
            [
                name
                for name, c in self.resource_clients.items()
                if c is not None and name not in self.stations_shutting_down
            ]
        )

        self.get_logger().info(
            f"[RC] Querying {self.total_expected_responses} resource agents..."
        )

        # Send async requests to all ResourceAgents
        for rname, client in self.resource_clients.items():

            if rname in self.stations_shutting_down:
                self.total_expected_responses -= 1
                continue

            if client is None:
                self.get_logger().warn(f"[RC] Skipping {rname}: client is None")
                self.total_expected_responses -= 1
                continue

            req = GetResources.Request()
            req.current_task = current_task
            req.product_type = product_type
            req.id = product_id

            fut = client.call_async(req)
            fut.add_done_callback(
                lambda future, rname=rname: self.resource_response_callback(
                    rname, future
                )
            )

        # Return immediately — DO NOT block!
        return response

    # ------------------ Factory initialization ------------------

    def launch_initial_factory_configuration(self):
        """Spawn initial stations and AGVs dynamically."""
        # Set skill times
        self.skillsTime["pickUp"] = 0.5
        self.skillsTime["drop"] = 0.5
        self.skillsTime["A"] = 4.0
        self.skillsTime["B"] = 4.0
        self.skillsTime["C"] = 5.0

        self.station_skills["Warehouse_Input"] = ["pickUp"]
        self.station_skills["Warehouse_Output"] = ["drop"]

        self.station_skills["Station1"] = ["A", "B"]

        self.station_skills["Station2"] = ["A", "C"]

        # Predefined positions
        pose_input = Pose2D(x=3.2, y=5.0, theta=0.0)
        pose_output = Pose2D(x=3.2, y=15.0, theta=0.0)
        location_list = [
            Pose2D(x=12.0, y=6.0, theta=0.0),
            Pose2D(x=18.0, y=6.0, theta=0.0),
        ]

        # Spawn warehouse input/output
        self.spawn_resource_agent("WarehouseInput", ["pickUp"], pose_input, [0.5])
        self.stationsLocation["WarehouseInput"] = pose_input
        self.create_get_resource_client("WarehouseInput")
        self.create_reserve_station_client("WarehouseInput")

        self.spawn_resource_agent("WarehouseOutput", ["drop"], pose_output, [0.5])
        self.stationsLocation["WarehouseOutput"] = pose_output
        self.create_get_resource_client("WarehouseOutput")
        self.create_reserve_station_client("WarehouseOutput")

        # Spawn initial stations and AGVs
        for i in [1, 2]:
            station_name = f"Station{i}"
            agv_name = f"AGV{i}"
            self.stationsLocation[station_name] = location_list[i - 1]
            self.get_logger().info(
                f"Resource: {station_name} and skills {self.station_skills.get(station_name)}"
            )
            self.spawn_resource_agent(
                station_name,
                self.station_skills.get(station_name),
                location_list[i - 1],
                [self.skillsTime[skill] for skill in self.station_skills[station_name]],
            )
            self.spawn_transport_agent(agv_name)
            # subscribe to occupied_transport for this AGV
            self.subscribe_to_agv_occupied(agv_name)
            # subscribe to amcl_pose for this AGV
            self.create_agv_pose_subscription(agv_name)
            # initialize state
            self.AGVs_status[agv_name] = False
            self.create_get_resource_client(station_name)
            self.create_reserve_station_client(station_name)

    # ------------------ Helpers ------------------

    def get_product_agents(self):
        """Return the namespaces of all current product agents."""
        node_names = self.get_node_names()
        product_names = [n for n in node_names if "product" in n]
        self.current_production = product_names
        return product_names

    def poses_equal(p1: Pose2D, p2: Pose2D, pos_tol=0.05) -> bool:
        if p1 is None or p2 is None:
            return False

        dx = p1.x - p2.x
        dy = p1.y - p2.y

        return math.hypot(dx, dy) < pos_tol


def main(args=None):
    rclpy.init(args=args)

    resource_controller = ResourceController()

    # use MultiThreadedExecutor instead of single-threaded spin()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(resource_controller)

    resource_controller.launch_initial_factory_configuration()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        resource_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

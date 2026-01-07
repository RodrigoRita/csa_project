import json
import math
import subprocess
import time

import rclpy
from csa_helper.msg import Product
from csa_helper.srv import GetProductTypes
from rclpy.node import Node
from std_msgs.msg import Bool, ColorRGBA


class OrderAgent(Node):

    def __init__(self, order_id: int):
        # Name of node
        node_name = f"order_{order_id}"
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter("order_id", 0)
        self.declare_parameter("product_list", rclpy.Parameter.Type.STRING_ARRAY)

        # Get launch parameters
        self.id = self.get_parameter("order_id").value
        self.order = (
            self.get_parameter("product_list").get_parameter_value().string_array_value
        )

        # Flags
        self.next_product = False
        self.product_types_loaded = False

        # Other variables and characteristics
        self.product_colors = {
            "A": ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            "B": ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
        }

        self.product_types = {}
        self.product_colors = {}

        # Publishers
        self.order_status_pub = self.create_publisher(Bool, f"/order_status", 10)

        # Clients
        self.product_type_client = self.create_client(
            GetProductTypes, "/get_product_types"
        )

        while not self.product_type_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /get_product_types service...")

    # Subscribers
    def create_next_product_subscriber(self, product, index):
        product_name = f"{product}_{index}"
        self.next_product_subscription = self.create_subscription(
            Bool, f"{product_name}/next_product", self.check_next_product, 10
        )

    # Listeners
    def check_next_product(self, msg):
        self.next_product = msg.data

    # Callback
    def product_types_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to get product types: {e}")
            return

        for product in res.products:
            self.product_types[product.name] = list(product.execution_plan)
            self.product_colors[product.name] = product.color

        self.product_types_loaded = True
        self.get_logger().info(
            f"Loaded product types: {list(self.product_types.keys())}"
        )

    # Launch node
    def launch_product_node(self, product, index):
        product_name = f"{product}_{index}"
        execution_plan_str = ",".join(self.product_types[self.order[index]])
        if not self.product_types or not self.product_colors:
            self.get_logger().warn("Product types not loaded yet")
            return
        subprocess.Popen(
            [
                "ros2",
                "run",
                "csa_project",
                "ProductAgent",
                "--ros-args",
                "-r",
                f"__node:={product_name}",
                "-p",
                f"product_id:={index}",
                "-p",
                f"product_type:={self.order[index]}",
                "-p",
                f"execution_plan:=[{execution_plan_str}]",
                "-p",
                f"color:=[{self.product_colors[self.order[index]].r},{self.product_colors[self.order[index]].g},{self.product_colors[self.order[index]].b},{self.product_colors[self.order[index]].a}]",
            ]
        )

    # Start sending products
    def execute_order(self):
        #
        completed = Bool()
        completed.data = False
        self.order_status_pub.publish(completed)

        # Request for products executionPlan and colors
        req = GetProductTypes.Request()
        future = self.product_type_client.call_async(req)
        future.add_done_callback(self.product_types_callback)

        # Wait until product types loaded
        while not self.product_types_loaded:
            rclpy.spin_once(self, timeout_sec=0.1)

        for i in range(len(self.order)):
            product_type = self.order[i]
            if i == 0:
                self.launch_product_node(product_type, i)
                self.create_next_product_subscriber(product_type, i)
            else:
                # Wait until next_product becomes True
                while not self.next_product:
                    rclpy.spin_once(self)  # process callbacks (e.g., subscriber)
                    time.sleep(0.1)  # avoid busy loop

                # reset the flag and launch next product
                self.next_product = False
                self.launch_product_node(product_type, i)
                self.create_next_product_subscriber(product_type, i)

        self.order_check_timer = self.create_timer(0.5, self.order_completed)

    def order_completed(self):
        node_names = self.get_node_names()
        product_names = [n for n in node_names if "product" in n]
        if not product_names:
            completed = Bool()
            completed.data = True
            self.order_status_pub.publish(completed)


def main(args=None):
    rclpy.init(args=args)

    # Create the OrderAgent node with a parameter for order_id
    node = OrderAgent(order_id=1)

    try:
        node.get_logger().info("OrderAgent started. Executing order...")
        node.execute_order()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("OrderAgent shutting down due to interrupt.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

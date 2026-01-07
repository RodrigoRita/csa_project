#!/usr/bin/env python3
import json
import subprocess
import sys
import threading

import rclpy
from csa_helper.msg import Agv, Order, Product, Station
from csa_helper.srv import GetProductTypes
from geometry_msgs.msg import Pose2D
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QApplication,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from std_msgs.msg import Bool, ColorRGBA, String

# Constant Global variables
MAX_PRODUCTS = 6
START_PRODUCTS = 2
TYPES = ["A", "B", "C", "D", "E", "F"]
MAX_STATIONS = 10
STATIONS = list(range(1, MAX_STATIONS + 1))
MAX_AGVS = 8
AGVS = list(range(1, MAX_AGVS + 1))


class FactoryGUI(Node):

    def __init__(self):
        super().__init__("factory_gui")
        # Product and Resources parameters
        self.id = -1  # Starts with 0 afterwards
        self.currentProducts = ["A", "B"]
        self.executionPlan = {
            "A": ["pickUp", "A", "B", "drop"],
            "B": ["pickUp", "A", "C", "drop"],
        }
        self.product_colors = {
            "A": ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0),
            "B": ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
        }
        self.currentStations = {"Station1": ["A", "B"], "Station2": ["A", "C"]}
        self.currentAgvs = [1, 2]
        self.currentTasks = {"A": 4.0, "B": 4.0, "C": 5.0}
        self.order = []
        self.rl_enabled = False  # initial state

        # ROS publishers
        self.order_pub = self.create_publisher(Order, "/new_order", 10)

        self.product_pub = self.create_publisher(Product, "/new_product", 10)
        self.product_delete_pub = self.create_publisher(String, "/delete_product", 10)

        self.station_pub = self.create_publisher(Station, "/new_station", 10)
        self.station_delete_pub = self.create_publisher(String, "/delete_station", 10)

        self.agv_pub = self.create_publisher(Agv, "/new_agv", 10)
        self.agv_delete_pub = self.create_publisher(String, "/delete_agv", 10)

        self.rl_status_pub = self.create_publisher(Bool, "/rl_enabled", 10)
        self.app = QApplication(sys.argv)
        self.window = QTabWidget()

        # ROS subscribers
        self.order_status_sub = self.create_subscription(
            Bool,
            "/order_status",
            self.order_status_callback,
            10,
        )

        # ROS Services
        self.get_product_types_srv = self.create_service(
            GetProductTypes,
            "/get_product_types",
            self.handle_get_product_types,
        )

        # Create all tabs
        self.order_tab = self.build_order_tab()
        self.product_tab = self.build_product_tab()
        self.resource_tab = self.build_resource_tab()
        self.transport_tab = self.build_transport_tab()

        self.window.addTab(self.order_tab, "Orders")
        self.window.addTab(self.product_tab, "Products")
        self.window.addTab(self.resource_tab, "Resources")
        self.window.addTab(self.transport_tab, "Transport")

        self.window.setWindowTitle("Factory Management Interface")
        self.window.resize(860, 600)
        self.window.show()

        # ------------------ Start ROS spinning in background thread ------------------
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()

    # Callbacks
    def product_callback(self, name, executionPlan, color):
        msg = Product()
        msg.name = name
        msg.execution_plan = executionPlan
        msg.color = color
        self.product_pub.publish(msg)
        self.get_logger().info('Publishing new product: "%s"' % msg.name)

    def station_callback(self, name, associatedSkills, skillsTime, location):
        msg = Station()
        msg.name = name
        msg.associated_skills = associatedSkills
        msg.skills_time = skillsTime
        msg.location = location
        self.station_pub.publish(msg)
        self.get_logger().info('Publishing new station: "%s"' % msg.name)

    def agv_callback(self, name):
        msg = Agv()
        msg.name = name
        self.agv_pub.publish(msg)
        self.get_logger().info('Publishing new agv: "%s"' % msg.name)

    def order_status_callback(self, msg):
        completed = msg.data
        if not completed:
            self.send_order_btn.setEnabled(False)
            self.order_status_label.setText("Order in progress")
            self.order_status_label.setStyleSheet("font-weight: bold; color: yellow;")
        else:
            self.send_order_btn.setEnabled(True)
            self.order_status_label.setText("Order completed")
            self.order_status_label.setStyleSheet("font-weight: bold; color: green;")

    def handle_get_product_types(self, request, response):
        """
        Returns all known product types with execution plans and colors
        """
        response.products = []

        for name, plan in self.executionPlan.items():
            p = Product()
            p.name = name
            p.execution_plan = plan
            p.color = self.product_colors[name]
            response.products.append(p)

        self.get_logger().info(
            f"Provided {len(response.products)} product types to OrderAgent"
        )

        return response

    # ---------------------------------------------
    #                    GUI
    # --------------------------------------------

    # ---------------- Order Tab ----------------
    def build_order_tab(self):
        self.order_tab = QWidget()
        layout = QVBoxLayout(self.order_tab)

        layout.setSpacing(15)  # <- Increase spacing between items
        layout.setContentsMargins(12, 12, 12, 12)

        # Store the number of active/available products
        self.num_products = START_PRODUCTS

        # Create 6 permanent input slots (disabled initially)
        self.order_slots = []
        self.order_slots_layout = QVBoxLayout()
        self.order_slots_layout.setSpacing(12)  # spacing between slots

        for i in range(MAX_PRODUCTS):
            line = QLineEdit()
            line.setPlaceholderText(f"Product {TYPES[i]}")
            line.setFixedHeight(30)
            line.setEnabled(i < START_PRODUCTS)  # enable first 2 only
            self.order_slots.append(line)
            self.order_slots_layout.addWidget(line)

        layout.addLayout(self.order_slots_layout)

        # Send Order button
        self.send_order_btn = QPushButton("Submit Order")
        self.send_order_btn.clicked.connect(self.send_order)
        self.send_order_btn.setEnabled(True)
        layout.addWidget(self.send_order_btn, alignment=Qt.AlignmentFlag.AlignRight)

        # Activate RL button
        self.RL_btn = QPushButton("RL")
        self.RL_btn.clicked.connect(self.send_rl)
        self.RL_btn.setEnabled(True)
        layout.addWidget(self.RL_btn, alignment=Qt.AlignmentFlag.AlignLeft)

        self.order_status_label = QLabel("No active order")
        self.order_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.order_status_label.setStyleSheet("font-weight: bold; color: gray;")
        layout.addWidget(self.order_status_label)

        self.order_tab.setLayout(layout)
        return self.order_tab

    def activate_next_order_slot(self):
        if self.num_products < MAX_PRODUCTS:
            self.order_slots[self.num_products].setEnabled(True)
            self.num_products += 1

    def update_order_slots(self):
        # Clear existing slots
        for i in reversed(range(self.order_slots_layout.count())):
            self.order_slots_layout.itemAt(i).widget().deleteLater()

        self.order_slots = []
        n = self.num_products_spin.value()

        for i in range(n):
            line = QLineEdit()
            line.setPlaceholderText(f"Product {TYPES[i+1]} quantity")
            self.order_slots.append(line)
            self.order_slots_layout.addWidget(line)

    def send_order(self):
        self.order = []  # Reset order list so that it is reseted at each order sent
        # Gather order data from GUI

        # --- Substitute to commented code bellow ---#
        for product, slot in zip(self.currentProducts, self.order_slots):
            if slot.text():
                self.order.extend([product] * int(slot.text()))

        if not self.order:
            self.get_logger().warn("No products in the order!")
            return

        # Publish ROS2 order message
        msg = Order()
        msg.order = self.order
        self.order_pub.publish(msg)
        self.get_logger().info(f"Sent order: {msg.order}")

        self.id += 1  # increment order id

        # Format product_list as ROS 2 STRING_ARRAY
        # Each string must be quoted for ROS 2 launch parameter
        ros_product_list = "[" + ",".join(f"'{p}'" for p in self.order) + "]"

        # Spawn the OrderAgent executable dynamically
        subprocess.Popen(
            [
                "ros2",
                "run",
                "csa_project",
                "OrderAgent",
                "--ros-args",
                "-p",
                f"order_id:={self.id}",
                "-p",
                f"product_list:={ros_product_list}",
            ]
        )
        self.get_logger().info(f"Spawned OrderAgent for order_id={self.id}")

    def send_rl(self):
        # Toggle state
        self.rl_enabled = not self.rl_enabled

        # Publish ROS message
        msg = Bool()
        msg.data = self.rl_enabled
        self.rl_status_pub.publish(msg)

        # Update button appearance
        if self.rl_enabled:
            self.RL_btn.setText("RL: ON")
            self.RL_btn.setStyleSheet(
                "background-color: green; color: white; font-weight: bold;"
            )
        else:
            self.RL_btn.setText("RL: OFF")
            self.RL_btn.setStyleSheet(
                "background-color: red; color: white; font-weight: bold;"
            )

        self.get_logger().info(f"RL enabled: {self.rl_enabled}")

    # ---------------- Product Tab ----------------
    def build_product_tab(self):
        tab = QWidget()
        form = QFormLayout()

        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("Only one uppercase letter, e.g: G")

        self.color_input = QLineEdit()
        self.color_input.setPlaceholderText("R,G,B,A")

        self.tasks_input = QLineEdit()
        self.tasks_input.setPlaceholderText("A;B;C;")

        add_btn = QPushButton("Add Product")
        add_btn.clicked.connect(self.add_product)
        delete_btn = QPushButton("Delete Product")
        delete_btn.clicked.connect(self.delete_product)

        form.addRow("Product name:", self.name_input)
        form.addRow("Color RGBA (e.g: 0.0,0.0,1.0,1.0):", self.color_input)
        form.addRow("Tasks:", self.tasks_input)
        form.addRow(add_btn)
        form.addRow(delete_btn)

        tab.setLayout(form)
        return tab

    def add_product(self):
        name = self.name_input.text().strip()
        tasks = self.tasks_input.text()
        colors = self.color_input.text()

        if not name or not colors or not tasks:
            print("Invalid product inputs.")
            return

        # Process tasks
        executionPlan = ["pickUp"]
        task_list = tasks.split(";")
        executionPlan.extend(task_list)
        executionPlan.append("drop")
        if not name.isupper():
            self.get_logger().warn(
                f"Product name {name} invalid, use one uppercase letter only!"
            )
            return
        if name in self.currentProducts:
            self.get_logger().warn(
                f"Product{name} already exists! For update, delete this product first."
            )
            return

        # Can the product be produced?
        missing_skills = [
            skill for skill in task_list if skill not in self.currentTasks
        ]
        if missing_skills:
            self.get_logger().warn(
                f"Product{name} cannot be produced, the factory lacks stations to execute the skills {', '.join(missing_skills)}."
            )
            return
        # Update executionPlan dict and currentProduct
        self.currentProducts.append(name)
        self.executionPlan[name] = executionPlan

        # Process color
        rgba = colors.split(",")
        try:
            r, g, b, a = map(float, rgba)
        except ValueError:
            self.get_logger().warn("Color must be R,G,B,A floats.")
            return

        color = ColorRGBA(r=r, g=g, b=b, a=a)
        self.product_colors[name] = color
        # Publish the new product to ROS2
        self.product_callback(name, executionPlan, color)
        # Activate the next order slot
        self.activate_next_order_slot()

        # Clear fields for next product
        self.color_input.clear()
        self.tasks_input.clear()

        print(f"Product type {name} added!")

    def delete_product(self):
        name = self.name_input.text().strip()
        if not name:
            print("Invalid product inputs for delete.")
            return

        if name not in self.currentProducts:
            self.get_logger().warn("Product does not exist!")
            return
        self.currentProducts.remove(name)
        # Update executionPlan dict
        self.executionPlan.pop(name, None)
        msg = String(data=name)
        self.product_delete_pub.publish(msg)

        # Clear fields for next product
        self.name_input.clear()
        print(f"Product type {name} deleted!")

        # Publish product removal
        msg = String(data=name)

    # ---------------- Resource Tab ----------------
    def build_resource_tab(self):
        tab = QWidget()
        form = QFormLayout()

        self.station_number_spin = QSpinBox()
        self.station_number_spin.setRange(1, MAX_STATIONS)

        self.resource_tasks_input = QLineEdit()
        self.resource_tasks_input.setPlaceholderText("A;4.0;B;4.0")

        self.resource_coord_input = QLineEdit()
        self.resource_coord_input.setPlaceholderText("x,y")

        add_btn = QPushButton("Add Station")
        add_btn.clicked.connect(self.add_station2list)
        del_btn = QPushButton("Delete Station")
        del_btn.clicked.connect(self.delete_station)

        form.addRow("Station number:", self.station_number_spin)
        form.addRow("Tasks;time:", self.resource_tasks_input)
        form.addRow("Coordinates:", self.resource_coord_input)
        form.addRow(add_btn)
        form.addRow(del_btn)

        tab.setLayout(form)
        return tab

    def add_station2list(self):
        number = self.station_number_spin.value()
        name = f"Station{number}"
        if name in self.currentStations.keys():
            self.get_logger().warn(f"{name} already exists!")
            return

        skills_time_list = self.resource_tasks_input.text().split(";")

        associatedSkills = []
        skillsTime = []
        for idx, value in enumerate(skills_time_list):
            if idx % 2 == 0:
                associatedSkills.append(value)
            else:
                skillsTime.append(float(value))

        # Check if there are new or repeated tasks. If a repeated task time value is different, request changing
        self.currentStations[name] = []  # Create empty array to append values
        for i in range(len(associatedSkills)):
            skill = associatedSkills[i]
            if skill in self.currentTasks.keys():
                if skillsTime[i] != self.currentTasks[skill]:
                    self.get_logger().warn(
                        f"Skill {skill}'s time is already existent and was wrongly assigned! To update delete the resource first"
                    )
                    self.currentStations.pop(
                        name, None
                    )  # Erase array, for it won't be used
                    return
            else:
                self.currentTasks[skill] = skillsTime[i]
            self.currentStations[name].append(skill)

        x_str, y_str = self.resource_coord_input.text().split(",")
        x, y = (
            float(x_str),
            float(y_str) - 0.45,
        )  # Use the coordinates set in the factory_world.yaml file and subtract 0.45 so that the goal is the station's AGV landing board
        location = Pose2D()
        location.x, location.y, location.theta = x, y, 0.0
        self.station_callback(
            name, associatedSkills, skillsTime, location
        )  # Aqui estás a colocar a GUI a publicar para os tópicos de novo recurso e novo agv para atualizar os resources. Vai ser subscripto pelo resource controller

        # Clear fields for next Resource
        self.resource_tasks_input.clear()
        self.resource_coord_input.clear()
        print(f"{name} added!")

    def delete_station(self):
        number = self.station_number_spin.value()
        name = f"Station{number}"
        if name not in self.currentStations.keys():
            self.get_logger().warn("Station does not exist!")
            return

        # Check what tasks are eliminated by this station deletion
        for task in self.currentStations[name]:
            stations = [
                station
                for station, skill in self.currentStations.items()
                if task in skill
            ]
            if (
                not stations
            ):  # If no stations can execute the skill, then the skill can't be executed by the factory
                self.currentTasks.pop(task, None)

        self.currentStations.pop(name)

        # Publish deleted station
        msg = String(data=name)
        self.station_delete_pub.publish(msg)

        # Clear fields for next Resource
        self.resource_tasks_input.clear()
        self.resource_coord_input.clear()
        print(f"Station{number} deleted!")

    # ---------------- Transport Tab ----------------
    def build_transport_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        self.agv_number_spin = QSpinBox()
        self.agv_number_spin.setRange(1, MAX_AGVS)

        btn = QPushButton("Summon New AGV")
        btn.clicked.connect(self.add_agv)
        layout.addWidget(btn)

        del_btn = QPushButton("Delete AGV")
        del_btn.clicked.connect(self.delete_agv)
        layout.addWidget(del_btn)

        layout.addWidget(self.agv_number_spin)
        tab.setLayout(layout)
        return tab

    def add_agv(self):
        number = self.agv_number_spin.value()
        if number in self.currentAgvs:
            self.get_logger().warn(f"AGV{number} already exists!")
            return
        name = f"AGV{number}"
        self.currentAgvs.append(number)
        self.agv_callback(name)

    def delete_agv(self):
        number = self.agv_number_spin.value()
        if number not in self.currentAgvs:
            self.get_logger().warn(f"AGV{number} doesn't exist!")
            return
        name = f"AGV{number}"
        self.currentAgvs.remove(number)

        # Publish deleted AGV
        msg = String(data=name)
        self.agv_delete_pub.publish(msg)

    # ---------------- Run ----------------
    def start(self):
        return self.app.exec()


def main():
    rclpy.init()
    gui = FactoryGUI()
    gui.start()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

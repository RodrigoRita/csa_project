import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from csa_helper.action import SelectPair
from rclpy.action import ActionServer
from rclpy.node import Node
from stable_baselines3 import PPO

MAX_STATIONS = 10
MAX_AGVS = 8

STATION_FEATURES = 3  # x, y, occupied
AGV_FEATURES = 3  # x, y, occupied


class RLScheduler(Node):

    def __init__(self):
        super().__init__("rl_scheduler")

        pkg_path = get_package_share_directory("csa_project")
        model_path = os.path.join(pkg_path, "rl_model", "factory_scheduler_ppo.zip")

        self.model = PPO.load(model_path, device="cpu")

        self.action_server = ActionServer(
            self, SelectPair, "select_pair", self.execute_callback
        )

    def execute_callback(self, goal_handle):
        g = goal_handle.request

        obs = self.build_observation(g)

        action, _ = self.model.predict(obs, deterministic=True)
        station_idx, agv_idx = int(action[0]), int(action[1])

        result = SelectPair.Result()

        # ---- Station selection ----
        if station_idx < len(g.station_names):
            result.chosen_station = g.station_names[station_idx]
        else:
            result.chosen_station = ""  # invalid → no-op

        # ---- AGV selection ----
        if agv_idx < len(g.agv_names):
            result.chosen_agv = g.agv_names[agv_idx]
        else:
            result.chosen_agv = ""

        goal_handle.succeed()
        return result

    def build_observation(self, g):
        obs = []

        # ---- Stations (pad or truncate) ----
        for i in range(MAX_STATIONS):
            if i < len(g.station_x):
                obs.extend(
                    [g.station_x[i], g.station_y[i], float(g.station_occupied[i])]
                )
            else:
                obs.extend([0.0, 0.0, 1.0])  # padded = unavailable

        # ---- AGVs (pad or truncate) ----
        for i in range(MAX_AGVS):
            if i < len(g.agv_x):
                obs.extend([g.agv_x[i], g.agv_y[i], float(g.agv_occupied[i])])
            else:
                obs.extend([0.0, 0.0, 1.0])  # padded = unavailable

        return np.array(obs, dtype=np.float32)


def main():
    rclpy.init()
    node = RLScheduler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

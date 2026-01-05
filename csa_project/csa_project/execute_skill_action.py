#!/usr/bin/env python3

import time

import rclpy
from csa_helper.action import ExecuteSkill
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class ExecuteSkillAction(Node):

    def __init__(self):
        super().__init__("execute_skill_action")

        self.action_server = ActionServer(
            self,
            ExecuteSkill,
            "execute_skill",
            self.execute_callback,
        )

        self.get_logger().info("ExecuteSkill Action Server ready")

    def execute_callback(self, goal_handle):
        goal = goal_handle.request

        self.get_logger().info(f"[RA] Executing skill '{goal.task}' on {goal.station}")

        # Skill duration
        duration = goal.skill_time
        start_time = time.time()

        feedback = ExecuteSkill.Feedback()

        while time.time() - start_time < duration:

            if goal_handle.is_cancel_requested:
                self.get_logger().warn("Skill canceled")

                goal_handle.canceled()

                result = ExecuteSkill.Result()
                result.success = False
                result.message = "Skill canceled"

                return result

            elapsed = time.time() - start_time
            feedback.state = f"Executing {goal.task} ({elapsed:.1f}s)"
            goal_handle.publish_feedback(feedback)

            time.sleep(0.1)

        goal_handle.succeed()

        result = ExecuteSkill.Result()
        result.success = True
        result.message = "Skill completed"

        self.get_logger().info("Skill completed successfully")

        return result


def main(args=None):
    rclpy.init(args=args)

    node = ExecuteSkillAction()

    # REQUIRED for blocking actions
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

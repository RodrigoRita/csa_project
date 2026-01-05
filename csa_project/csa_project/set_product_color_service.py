#!/usr/bin/env python3

import rclpy
from csa_helper.srv import SetProductColor
from rclpy.node import Node
from std_msgs.msg import ColorRGBA


class ProductBodyService(Node):
    __slots__ = ("publishers",)

    def __init__(self):
        super().__init__("product_body_service")

        self.publishers = {}

        self.srv = self.create_service(
            SetProductColor,
            "set_product_body",
            self.callback,
        )

        self.get_logger().info("ProductBody service ready")

    def get_publisher(self, resource_name):
        if resource_name not in self.publishers:
            self.publishers[resource_name] = self.create_publisher(
                ColorRGBA,
                f"{resource_name}/product_color",
                10,
            )
        return self.publishers[resource_name]

    def callback(self, request, response):
        if not request.resource_name:
            self.get_logger().warn("Empty resource_name → ignoring color update")
            response.success = False
            return response

        pub = self.get_publisher(request.resource_name)

        if request.enable:
            pub.publish(request.color)
        else:
            pub.publish(ColorRGBA())

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ProductBodyService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

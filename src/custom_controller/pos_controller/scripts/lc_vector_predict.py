#!/usr/bin/env python3
from custom_controller_interfaces.srv import VectorPredictionLC
from custom_controller_interfaces.msg import LCMsg
from custom_controller_interfaces.msg import VecPredictMsg

import rclpy
from rclpy.node import Node


class PythonNode(Node):

    def __init__(self):
        super().__init__("calculate_vector")
        self.srv = self.create_service(VectorPredictionLC, "vector_prediction_lc", self.calculate_vector_lc)
        self.prediction_publisher = self.create_publisher(VecPredictMsg, "vec_predict", 10)
        self.lcmsg_subscription = self.create_subscription(
            LCMsg,
            "LC_msg",
            self.listener_callback,
            10)
        self.lcmsg_subscription

    def listener_callback(self, msg):
        print(msg.pos_x)
        msg_back = VecPredictMsg()
        msg_back.x = 1.234
        self.prediction_publisher.publish(msg_back)

    def calculate_vector_lc(self, request, response):
        self.get_logger().info("bazinger")
        print(request)
        response.x = 2.1
        return response


def main(args=None):
    rclpy.init(args=args)

    python_node = PythonNode()

    rclpy.spin(python_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
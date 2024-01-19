from custom_controller_interfaces.srv import VectorPredictionLC

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('calculate_vector')
        self.srv = self.create_service(CalculateVectorLC, 'vector_prediction_lc', self.calculate_vector_lc)

    def calculate_vector_lc(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
from demo.levensthein import LevenshteinService
from interfaces.srv import LevenstheinSrv

import rclpy
from rclpy.node import Node


class Service(Node):

    def __init__(self, levensthein_service: LevenshteinService):
        super().__init__("service")

        self.levensthein_service = levensthein_service

        self.srv = self.create_service(
            LevenstheinSrv,
            "get_levensthein_distance",
            self.get_levensthein_distance_callback,
        )

        self.get_logger().info("Service running")

    def get_levensthein_distance_callback(
        self, request: LevenstheinSrv.Request, response: LevenstheinSrv.Response
    ):
        word1: str = request.word1
        word2: str = request.word2

        # Surprisingly, ROS interfaces are not type-safe
        if type(word1) != type(""):
            raise Exception
        if type(word2) != type(""):
            raise Exception

        response.distance = self.levensthein_service.calculate(word1, word2)

        return response


def main(args=None):
    rclpy.init(args=args)
    levensthein = LevenshteinService()
    minimal_service = Service(levensthein)
    rclpy.spin(minimal_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

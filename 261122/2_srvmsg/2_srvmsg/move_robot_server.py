import rclpy
from rclpy.node import Node
from custom_interfaces.srv import MoveRobot
import math

class MoveRobotServer(Node):
    def __init__(self):
        super().__init__('srv_server')

        self.srv = self.create_service(MoveRobot, '/move_robot', self.srv_callback)
        self.get_logger().info('Srv Server Ready.')

    def srv_callback(self, request, response):
        x = request.current_pose.x
        y = request.current_pose.y
        theta = request.current_pose.theta
        distance = request.distance
        math.cos(theta)
        math.sin(theta)

        response.result_pose.x = x + math.cos(theta) * request.distance
        response.result_pose.y = y + math.sin(theta) * request.distance
        response.result_pose.theta = theta
        response.success = True
        
        self.get_logger().info(f'Incoming request\nx: {request.current_pose.x}, y: {request.current_pose.y}, theta: {request.current_pose.theta}')
        self.get_logger().info(f'Sending back response: [{response.result_pose.x}, {response.result_pose.y}, {response.result_pose.theta}]')
        
        return response


def main():
    rclpy.init()
    node = MoveRobotServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

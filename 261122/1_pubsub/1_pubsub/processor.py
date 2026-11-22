import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np

class Processor(Node):
    def __init__(self):
        super().__init__('processor')

        self.subscription_ = self.create_subscription(String, '/wasd', self.callback, 10)
        self.subscription_
        self.publisher_ = self.create_publisher(String, '/location', 10)

        self.loc = np.array([0,0])

    def callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        prev = self.loc.copy()
        match msg.data:
            case "w":
                self.loc += [0, 1]
            case "a":
                self.loc += [-1, 0]
            case "s":
                self.loc += [0, -1]
            case "d":
                self.loc += [1, 0]
            case _:
                self.get_logger().warn(f'Unknown key: "{msg.data}"')
                return
        self.publish_message(prev)


    def publish_message(self, prev):
        msg = String()
        msg.data = f"({prev[0], prev[1]}) => ({self.loc[0], self.loc[1]})"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        

def main():
    rclpy.init()
    node = Processor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

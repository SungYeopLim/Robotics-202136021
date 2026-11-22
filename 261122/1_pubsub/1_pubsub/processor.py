import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import numpy as np

class Processor(Node):
    def __init__(self):
        super().__init__('processor')

        self.subscription_ = self.create_subscription(String, '/location', self.callback, 10)
        self.subscription_
        self.publisher_ = self.create_publisher(String, '/location', 10)

        self.start_time = time.time()
        self.timer = self.create_timer(1.0, self.publish_message)

    def callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        self.loc = np.array([0, 0])
        match msg.data:
            case "w":
                self.loc += [0, 1]
            case "a":
                self.loc += [-1, 0]
            case "s":
                self.loc += [0, -1]
            case "d":
                self.loc += [1, 0]


    def publish_message(self):
        elapsed = int(time.time() - self.start_time)
        msg = String()
        msg.data = f"({self.loc})"
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

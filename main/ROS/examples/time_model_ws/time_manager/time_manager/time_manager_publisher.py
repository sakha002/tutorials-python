
import rclpy
from rclpy.node import Node

from rclpy.clock import Clock
from rclpy.time import Time

from builtin_interfaces.msg import Time



# from std_msgs.msg import String


class TimePublisher(Node):

    def __init__(self):
        super().__init__('time_publisher')
        self.publisher_ = self.create_publisher(Time, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        message = Clock().now().to_msg()

       
        # msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(message)
        self.get_logger().info('Publishing Time for : "%s"' % message)
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)

    publisher = TimePublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

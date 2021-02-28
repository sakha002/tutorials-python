import rclpy
from rclpy.node import Node

# from rclpy.time import Time
# from rclpy.clock import Clock

from rcl_interfaces.builtin_interfaces.msg import Time

class TimeSubscriber(Node):

    def __init__(self):
        super().__init__('time_subscriber')
        self.subscription = self.create_subscription(
            Time,                                             
            'topic',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
            self.get_logger().info('I heard time as: "%s"' % str(msg) ) # CHANGE
            mssg=  str(msg)
            self.get_logger().info( mssg)



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = TimeSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
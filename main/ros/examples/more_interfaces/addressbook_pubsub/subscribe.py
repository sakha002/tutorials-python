import rclpy
from rclpy.node import Node

from addressbook_msg.msg import AddressBook        # CHANGE


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            AddressBook,                                              # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
            self.get_logger().info('I heard: "%s"' % msg.first_name) # CHANGE
            mssg= msg.last_name +',,,' + str(msg.age) + str(msg.address.city)
            self.get_logger().info( mssg)



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
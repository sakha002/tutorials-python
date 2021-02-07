
import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from addressbook_msg.msg import AddressBook


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(AddressBook, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        message = AddressBook()

        message.first_name = "John"
        message.last_name = "Doe"
        message.age = 30
        message.gender = message.MALE
        message.address.city = "unknown"
        message.address.street= '120'

        # msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(message)
        self.get_logger().info('Publishing Address for : "%s"' % message.first_name)
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node

from rclpy.clock import Clock, ROSClock
from rclpy.time import Time
from rclpy.time_source import TimeSource
from builtin_interfaces.msg import Time
from rclpy.parameter import Parameter

import rosgraph_msgs.msg


# from std_msgs.msg import String


class TimePublisher(Node):

    def __init__(self):
        super().__init__('time_publisher')
        self.publisher_ = self.create_publisher(rosgraph_msgs.msg.Clock, '/clock', 10)
        timer_period = 2.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        # message = Clock().now().to_msg()
        cycle_count = 0

        # while cycle_count < 1000:

        time_msg = rosgraph_msgs.msg.Clock()
        time_msg.clock.sec = self.i



        
            # msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(time_msg)
        self.get_logger().info('Publishing Time for : "%s"' % time_msg)
        self.i += 10
            # cycle_count += 10



def main(args=None):
    rclpy.init(args=args)

    publisher = TimePublisher()

    rclpy.spin(publisher)


    # time_source = TimeSource(node=publisher)

    #     # ROSClock is a specialization of Clock with ROS time methods.
    # time_source.attach_clock(ROSClock())

    publisher.set_parameters(
            [Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    


    # print(time_source.ros_time_is_active)




    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import math

import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MinimalDriver(Node):
    def __init__(self):
        super().__init__('minimal_driver')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # 10 Hz, thus 0.1
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.i = 0
        self.state = 0

        self.angular_rate = (90 * (math.pi / 180))

    def timer_callback(self):
        msg = Twist()
        self.i += 1

        if self.state == 2:
            msg.linear.x = 0.5
        elif self.state == 4:
            msg.angular.z = self.angular_rate
        # msg.linear.x = 0.1
        # msg.angular.z = 1.0
        self.publisher.publish(msg)
        print("Squaring: {}, {}, {}".format(self.i % 10, self.state, self.i), end="\r", flush=True)
        if self.i == 11:
            self.state = (self.state + 1) if self.state != 4 else 0
            # self.state = 0 if self.state == 1 else 1
            self.i = 0

def main(args=None):
    rclpy.init(args=args)
    minimal_driver = MinimalDriver()

    rclpy.spin(minimal_driver)
    minimal_driver.destroy_node()
    rclpy.shutdown() 

if __name__ == "__main__":
    main()
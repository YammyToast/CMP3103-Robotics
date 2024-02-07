import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MinimalDriver(Node):
    def __init__(self):
        super.__init__('minimal_driver')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # 10 Hz, thus 0.1
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 1

def main(args=None):
    rclpy.init(args=args)
    minimal_driver = MinimalDriver()

    rclpy.spin(minimal_driver)
    minimal_driver.destroy_node()
    rclpy.shutdown() 

if __name__ == "__main__":
    main()
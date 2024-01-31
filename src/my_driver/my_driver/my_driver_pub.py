import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from dataclasses import dataclass, KW_ONLY

@dataclass
class CommandVec:
    """Command Vector Dataclass."""
    _: KW_ONLY
    l_x: float
    l_y: float
    l_z: float
    a_x: float
    a_y: float
    a_z: float

    def serialize(self) -> str:
        s_buf = []
        s_buf.append("linear:")
        s_buf.append(f"  x:{self.l_x}")
        s_buf.append(f"  y:{self.l_y}")
        s_buf.append(f"  z:{self.l_z}")
        s_buf.append("angular:")
        s_buf.append(f"  x:{self.a_x}")
        s_buf.append(f"  y:{self.a_y}")
        s_buf.append(f"  z:{self.a_z}")
        return "\n".join(s_buf)


class TeleOperator(Node):
    def __init__(self):
        print("temp")
        super().__init__('my_driver_subscription')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.i = 0
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello World: {self.i}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.i += 1

    def publish_command(__command: CommandVec):
        print("temp")


def main(args=None):
    rclpy.init()
    
    tele_op = TeleOperator()
    command = CommandVec(
        l_x= 0.1,
        l_y= 0,
        l_z= 0,
        a_x= 0,
        a_y= 0,
        a_z= 1)
    print(command.serialize())

    rclpy.spin(tele_op)
    tele_op.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

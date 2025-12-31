import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobotController(Node):
    def __init__(self):
        Node.__init__(self, 'robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_robot)
        self.get_logger().info('Robot controller node instantiated')

    def move_robot(self):
        msg = Twist()
        msg.linear.x = 0.1
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.publisher_.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



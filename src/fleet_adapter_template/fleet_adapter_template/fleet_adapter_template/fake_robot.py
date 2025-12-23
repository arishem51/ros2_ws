import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import RobotState, Location

class FakeRobot(Node):
    def __init__(self):
        super().__init__("fake_robot")
        self.pub = self.create_publisher(RobotState, "robot_state", 10)
        self.timer = self.create_timer(0.1, self.publish_state)

    def publish_state(self):
        msg = RobotState()
        msg.name = "cleanerBotA_1"
        msg.model = "cleanerBotA"

        msg.location = Location()
        msg.location.level_name = "L1"
        msg.location.x = 26.364960871641816
        msg.location.y = -27.752566946910367
        msg.location.yaw = 0.0

        self.pub.publish(msg)
        self.get_logger().info(f"Published robot_state: x={msg.location.x}, y={msg.location.y}")

def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
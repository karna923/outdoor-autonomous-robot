import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

STOP_DISTANCE_M = 0.5
WARN_DISTANCE_M = 1.0
TURN_VEL = 0.4
LINEAR_VEL_MAX = 0.5


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.latest_range = None
        self.avoiding = False

        self.range_sub = self.create_subscription(
            Range,
            '/ultrasonic_range',
            self.range_callback,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel_safe',
            10
        )

        self.get_logger().info('Obstacle avoidance node started')

    def range_callback(self, msg: Range):
        self.latest_range = msg.range

    def cmd_vel_callback(self, nav2_cmd: Twist):
        out = Twist()

        if self.latest_range is None:
            self.cmd_vel_pub.publish(nav2_cmd)
            return

        if self.latest_range < STOP_DISTANCE_M:
            self.avoiding = True
            out.linear.x = 0.0
            out.angular.z = TURN_VEL
            self.get_logger().warn(
                f'Obstacle at {self.latest_range:.2f}m -- overriding cmd_vel'
            )

        elif self.latest_range < WARN_DISTANCE_M:
            scale = (self.latest_range - STOP_DISTANCE_M) / (WARN_DISTANCE_M - STOP_DISTANCE_M)
            out.linear.x = nav2_cmd.linear.x * scale
            out.angular.z = nav2_cmd.angular.z
            self.avoiding = False

        else:
            out.linear.x = nav2_cmd.linear.x
            out.angular.z = nav2_cmd.angular.z
            self.avoiding = False

        self.cmd_vel_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

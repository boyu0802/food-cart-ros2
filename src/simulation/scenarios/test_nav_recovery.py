"""
Test scenario: Obstacle blocks planned path - Nav2 recovery behaviors.

A large obstacle (actor or box) blocks the hallway. Tests whether
Nav2's recovery behaviors (spin, backup, costmap clearing) allow
the robot to find an alternate path.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Int32


class NavRecoveryTest(Node):
    def __init__(self):
        super().__init__('nav_recovery_test')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub    = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.create_subscription(Int32, '/elevator/floor', self.floor_cb, 10)

        self.current_floor = None

        self.timer = self.create_timer(2.0, self.run_test)
        self.test_step  = 0
        self.start_time = self._now()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def floor_cb(self, msg):
        self.current_floor = msg.data

    def run_test(self):
        elapsed = self._now() - self.start_time

        if self.test_step == 0:
            self.get_logger().info('=== TEST: NAV RECOVERY ===')
            self.get_logger().info('Blocked path should trigger Nav2 recovery behaviors')
            self.get_logger().info('Goal: west end of hallway (past obstacles)')
            self.test_step = 1
            self.start_time = self._now()

        elif self.test_step == 1:
            self.get_logger().info('Publishing Nav2 goal...')
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = -12.0
            goal.pose.position.y = 0.0
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.test_step = 2
            self.start_time = self._now()

        elif self.test_step == 2 and elapsed > 20.0:
            self.get_logger().info('If still blocked: trying alternative path')
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = -12.0
            goal.pose.position.y = -1.5
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.test_step = 3
            self.start_time = self._now()

        elif self.test_step == 3 and elapsed > 40.0:
            self.get_logger().info('=== NAV RECOVERY TEST COMPLETE ===')
            self.get_logger().info('Check Nav2 logs for recovery behavior usage')
            self.destroy_timer(self.timer)


def main():
    rclpy.init()
    node = NavRecoveryTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

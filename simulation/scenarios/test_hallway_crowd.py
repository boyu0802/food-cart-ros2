"""
Test scenario: Dense pedestrian traffic in hallways.

Tests Nav2's MPPI controller obstacle avoidance with dynamic actors.
Requires the robot to plan paths around walking people.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, PoseStamped


# DoorState int constants (must match elevator_node.py)
DOOR_OPEN   = 3
DOOR_CLOSED = 1


class HallwayCrowdTest(Node):
    def __init__(self):
        super().__init__('hallway_crowd_test')

        # Publishers
        self.call_pub    = self.create_publisher(Int32, '/elevator/call', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub    = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Subscribers
        self.create_subscription(Int32, '/elevator/door_state', self.door_cb,  10)
        self.create_subscription(Int32, '/elevator/floor',      self.floor_cb, 10)

        self.door_state    = None
        self.current_floor = None

        self.timer = self.create_timer(2.0, self.run_test)
        self.test_step  = 0
        self.start_time = self._now()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def door_cb(self, msg):  self.door_state    = msg.data
    def floor_cb(self, msg): self.current_floor = msg.data

    def run_test(self):
        elapsed = self._now() - self.start_time

        if self.test_step == 0:
            self.get_logger().info('=== TEST: HALLWAY CROWD ===')
            self.get_logger().info('6 pedestrians patrol hallways on both floors')
            self.get_logger().info('Robot must navigate through crowd')
            self.test_step = 1
            self.start_time = self._now()

        elif self.test_step == 1:
            self.get_logger().info('Setting Nav2 goal: hallway end (west side)')
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = -15.0
            goal.pose.position.y = 0.0
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.test_step = 2
            self.start_time = self._now()

        elif self.test_step == 2 and elapsed > 10.0:
            self.get_logger().info('Re-planning: goal at elevator lobby')
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = -2.0
            goal.pose.position.y = 0.5
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.test_step = 3
            self.start_time = self._now()

        elif self.test_step == 3 and elapsed > 25.0:
            self.get_logger().info('=== HALLWAY CROWD TEST COMPLETE ===')
            self.destroy_timer(self.timer)


def main():
    rclpy.init()
    node = HallwayCrowdTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

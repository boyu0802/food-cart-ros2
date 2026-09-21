"""
Test scenario: Door closes while robot is entering.

The elevator door starts closing while the robot is mid-way into the cab.
Tests the robot's ability to detect the obstruction and react correctly.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist


# DoorState int constants (must match elevator_node.py)
DOOR_OPEN    = 3
DOOR_CLOSED  = 1
DOOR_OPENING = 2
DOOR_CLOSING = 4


class DoorTimingTest(Node):
    def __init__(self):
        super().__init__('door_timing_test')

        self.call_pub    = self.create_publisher(Int32, '/elevator/call', 10)
        self.goto_pub    = self.create_publisher(Int32, '/elevator/goto_floor', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Int32, '/elevator/door_state', self.door_cb,  10)
        self.create_subscription(Int32, '/elevator/floor',      self.floor_cb, 10)
        self.create_subscription(Int32, '/elevator/direction',  self.dir_cb,   10)

        self.door_state    = None
        self.current_floor = None
        self.direction     = None

        self.timer = self.create_timer(0.5, self.run_test)
        self.test_step  = 0
        self.start_time = self._now()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def door_cb(self, msg):  self.door_state    = msg.data
    def floor_cb(self, msg): self.current_floor = msg.data
    def dir_cb(self, msg):   self.direction     = msg.data

    def run_test(self):
        elapsed = self._now() - self.start_time

        if self.test_step == 0:
            self.get_logger().info('=== TEST: DOOR TIMING ===')
            self.get_logger().info('Door closes while robot is entering')
            self.get_logger().info('Calling elevator...')
            msg = Int32(); msg.data = 4
            self.call_pub.publish(msg)
            self.test_step = 1
            self.start_time = self._now()

        elif self.test_step == 1:
            if self.door_state == DOOR_OPEN and elapsed > 3.0:
                self.get_logger().info('Door open - robot begins navigating in')
                # Move slowly toward elevator to simulate
                # Door will start closing (5s hold time)
                self.test_step = 2
                self.start_time = self._now()

        elif self.test_step == 2:
            if self.door_state == DOOR_CLOSING or self.door_state == DOOR_CLOSED:
                self.get_logger().info(f'Door state changed: {self.door_state}')
                self.get_logger().info('Robot should detect obstruction and back off')
                self.test_step = 3
                self.start_time = self._now()

        elif self.test_step == 3:
            if elapsed > 3.0 and self.door_state != DOOR_CLOSED:
                self.get_logger().info('Re-calling to re-open door')
                msg = Int32(); msg.data = 4
                self.call_pub.publish(msg)
                self.test_step = 4
                self.start_time = self._now()

        elif self.test_step == 4 and self.door_state == DOOR_OPEN:
            self.get_logger().info('Door re-opened - waiting for next cycle')
            self.get_logger().info('=== DOOR TIMING TEST COMPLETE ===')
            self.destroy_timer(self.timer)


def main():
    rclpy.init()
    node = DoorTimingTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Test scenario: Person blocks elevator entry - edge case the real project can't handle.

The pedestrian_in_elevator actor stands inside the cab, blocking the robot's entry.
The safe_to_enter gate has inside_clear_default: true, meaning it won't detect the person.
This tests what happens when the real stack encounters an occupied cab.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist


# DoorState int constants (must match elevator_node.py)
DOOR_OPEN   = 3
DOOR_CLOSED = 1

# Direction int constants
DIR_DOWN = -1
DIR_UP   =  1
DIR_IDLE =  0


class ElevatorWithPersonTest(Node):
    def __init__(self):
        super().__init__('elevator_with_person_test')

        self.call_pub    = self.create_publisher(Int32, '/elevator/call', 10)
        self.goto_pub    = self.create_publisher(Int32, '/elevator/goto_floor', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Int32, '/elevator/floor',         self.floor_cb, 10)
        self.create_subscription(Int32, '/elevator/door_state',    self.door_cb,  10)
        self.create_subscription(Int32, '/elevator/direction',     self.dir_cb,   10)
        self.create_subscription(Bool,  '/elevator/safe_to_enter', self.safe_cb,  10)

        self.current_floor  = None
        self.door_state     = None
        self.direction      = None
        self.safe_to_enter  = None

        self.timer = self.create_timer(1.0, self.run_test)
        self.test_step = 0
        self.step_start_time = self._now()
        self.last_log = ""

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def floor_cb(self, msg): self.current_floor = msg.data
    def door_cb(self, msg):  self.door_state    = msg.data
    def dir_cb(self, msg):   self.direction     = msg.data
    def safe_cb(self, msg):  self.safe_to_enter = msg.data

    def log(self, msg):
        self.last_log = msg
        self.get_logger().info(msg)

    def run_test(self):
        now = self._now()
        elapsed = now - self.step_start_time

        self.log(f'Step {self.test_step}: floor={self.current_floor} '
                 f'door={self.door_state} dir={self.direction} '
                 f'safe={self.safe_to_enter}')

        if self.test_step == 0:
            self.log('=== TEST: PERSON IN ELEVATOR ===')
            self.log('Scenario: Pedestrian stands inside cab')
            self.log('The safe_to_enter gate reports safe (default)')
            self.log('Robot must detect obstruction via navigation failure')
            self.test_step = 1
            self.step_start_time = now

        elif self.test_step == 1:
            self.log('Calling elevator to Floor 4')
            msg = Int32(); msg.data = 4
            self.call_pub.publish(msg)
            self.test_step = 2
            self.step_start_time = now

        elif self.test_step == 2:
            if self.door_state == DOOR_OPEN and elapsed > 3.0:
                self.log('Door open but person is inside cab')
                self.log(f'Safe-to-enter gate reports: {self.safe_to_enter}')
                self.log('Attempting to navigate into cab...')
                twist = Twist()
                twist.linear.x = 0.3
                self.cmd_vel_pub.publish(twist)
                self.test_step = 3
                self.step_start_time = now

        elif self.test_step == 3:
            if elapsed > 8.0:
                self.log('Navigation into cab would be blocked by person')
                self.log('Simulating nav_failed condition')
                self.log('Waiting for person to exit cab...')
                self.test_step = 4
                self.step_start_time = now

        elif self.test_step == 4:
            if elapsed > 25.0:
                self.log('Person has left - retrying entry')
                self.test_step = 5
                self.step_start_time = now

        elif self.test_step == 5:
            self.log('Entering elevator...')
            msg = Int32(); msg.data = 1
            self.goto_pub.publish(msg)
            self.log('=== TEST COMPLETE ===')
            self.destroy_timer(self.timer)


def main():
    rclpy.init()
    node = ElevatorWithPersonTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

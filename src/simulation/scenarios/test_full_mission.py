"""
Test scenario: Full mission - Floor 4 pickup -> elevator -> Floor 1 dropoff
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool


# DoorState int constants (must match elevator_node.py)
DOOR_OPEN   = 3
DOOR_CLOSED = 1

# Direction int constants
DIR_DOWN = -1
DIR_IDLE =  0


class FullMissionTest(Node):
    def __init__(self):
        super().__init__('full_mission_test')

        self.call_pub = self.create_publisher(Int32, '/elevator/call', 10)
        self.goto_pub = self.create_publisher(Int32, '/elevator/goto_floor', 10)

        # State tracking
        self.current_floor  = None
        self.door_state     = None
        self.direction      = None
        self.safe_to_enter  = None

        self.create_subscription(Int32, '/elevator/floor',        self.floor_cb, 10)
        self.create_subscription(Int32, '/elevator/door_state',   self.door_cb,  10)
        self.create_subscription(Int32, '/elevator/direction',    self.dir_cb,   10)
        self.create_subscription(Bool,  '/elevator/safe_to_enter',self.safe_cb,  10)

        self.timer = self.create_timer(1.0, self.run_test)
        self.test_step = 0
        self.step_start_time = self._now()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def floor_cb(self, msg): self.current_floor = msg.data
    def door_cb(self, msg):  self.door_state    = msg.data
    def dir_cb(self, msg):   self.direction     = msg.data
    def safe_cb(self, msg):  self.safe_to_enter = msg.data

    def run_test(self):
        now = self._now()

        if self.test_step == 0:
            self.get_logger().info('=== FULL MISSION TEST START ===')
            self.get_logger().info('Step 0: Robot starts at Floor 4 kitchen (pickup zone)')
            self.test_step = 1
            self.step_start_time = now

        elif self.test_step == 1:
            if self.current_floor == 4:
                self.get_logger().info('Confirmed at Floor 4')
                self.get_logger().info('Step 1: Call elevator from hallway')
                msg = Int32(); msg.data = 4
                self.call_pub.publish(msg)
                self.test_step = 2
                self.step_start_time = now

        elif self.test_step == 2:
            if self.door_state == DOOR_OPEN and self.safe_to_enter:
                self.get_logger().info('Step 2: Elevator door open and safe - navigate into cab')
                self.test_step = 3

        elif self.test_step == 3:
            self.get_logger().info('Step 3: In cab - press Floor 1 button')
            msg = Int32(); msg.data = 1
            self.goto_pub.publish(msg)
            self.test_step = 4

        elif self.test_step == 4:
            if self.direction == DIR_DOWN:
                self.get_logger().info('Step 4: Elevator moving down')
                self.test_step = 5

        elif self.test_step == 5:
            if self.current_floor == 1 and self.door_state == DOOR_OPEN:
                self.get_logger().info('Step 5: Arrived at Floor 1 - exit elevator')
                self.test_step = 6

        elif self.test_step == 6:
            self.get_logger().info('Step 6: Navigate to dropoff zone (classroom area)')
            self.get_logger().info('=== MISSION COMPLETE ===')
            self.destroy_timer(self.timer)


def main():
    rclpy.init()
    node = FullMissionTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

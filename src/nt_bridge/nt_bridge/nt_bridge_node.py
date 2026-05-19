#!/usr/bin/env python3
"""NetworkTables 4 <-> ROS2 bridge for Team 6998 autonomous food cart.

Five flows:
  ROS2  /cmd_vel          ->  NT  Nav/cmd/{vx,vy,omega,heartbeat,timestamp}
  NT    Robot/odom/*      ->  ROS2 nav_msgs/Odometry on /odom (+ optional TF)
  NT    Robot/imu/*       ->  ROS2 sensor_msgs/Imu on /imu
  NT    limelight/{tv,tid,tx,ty,ta,targetpose_robotspace}
                          ->  ROS2 cart_elevator_msgs/TagDetection on /limelight/tag

Mission control (new 2026-05-19/20):
  ROS2  /elevator/press_button       ->  NT  Pi/elevator/press_button (string)
  ROS2  /mission/cmd                 ->  NT  Pi/mission/cmd (string,
                                             "load" | "unload" | "stow")
  NT    Robot/elevator/press_done    ->  ROS2 std_msgs/Bool on /elevator/press_done
  NT    Robot/mission/start_pressed  ->  ROS2 std_msgs/Bool on /mission/start
  NT    Robot/mission/loaded         ->  ROS2 std_msgs/Bool on /mission/loaded
  NT    Robot/mission/unloaded       ->  ROS2 std_msgs/Bool on /mission/unloaded
  NT    Robot/mission/enable         ->  ROS2 std_msgs/Bool on /mission/enable
                                         (level — held while operator's hold-to-run
                                         button is pressed; republished on change)
  NT    Robot/mission/restart_pressed -> ROS2 std_msgs/Bool on /mission/restart
                                         (edge — rising edge each restart press)

NT->ROS bools are edge-triggered: we publish Bool(True) the first tick we
see the NT value go False->True, then nothing until the next rising edge.
That matches the supervisor's latch semantics — it only cares about "the
event happened," not the held state. Polling at mission_in_rate_hz; pick
high enough that a 1-robot-tick (~20 ms) FRC pulse can't slip through.

Exception: `/mission/enable` is level-triggered. We republish it (with
transient_local QoS so late subscribers get the current value) on every
False->True or True->False change. The supervisor checks the current
value at tick time, so it must be a held bool not an edge.

Watchdog: if no /cmd_vel arrives within `cmd_vel_max_age` seconds, the bridge
publishes (0, 0, 0) to NT so the RoboRIO stops the swerve.
"""

import math
from threading import Lock
from typing import Optional

import ntcore
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster

from cart_elevator_msgs.msg import TagDetection


LARGE_COV = 1e6


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def rpy_deg_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """ZYX (yaw, pitch, roll) intrinsic Euler in degrees -> quaternion (x, y, z, w).
    Matches the convention Limelight uses for targetpose_robotspace."""
    r = math.radians(roll_deg) * 0.5
    p = math.radians(pitch_deg) * 0.5
    y = math.radians(yaw_deg) * 0.5
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return (qx, qy, qz, qw)


class NtBridge(Node):
    def __init__(self) -> None:
        super().__init__('nt_bridge')

        self.declare_parameters(namespace='', parameters=[
            ('team_number', 6998),
            ('server_address', ''),
            ('nt_client_name', 'orange_pi_nav'),
            ('enable_cmd_vel_out', True),
            ('enable_odom_in', True),
            ('enable_imu_in', True),
            ('enable_limelight_in', True),
            ('enable_mission_io', True),
            ('cmd_vel_topic', '/cmd_vel'),
            ('cmd_vel_stamped', False),
            ('cmd_vel_max_age', 0.5),
            ('odom_topic', '/odom'),
            ('imu_topic', '/imu'),
            ('tag_topic', '/limelight/tag'),
            ('nt_cmd_table', 'Nav/cmd'),
            ('nt_odom_table', 'Robot/odom'),
            ('nt_imu_table', 'Robot/imu'),
            ('nt_limelight_table', 'limelight'),
            # Mission-IO NT keys (flat — no shared sub-table) so they
            # show up cleanly in Shuffleboard / OutlineViewer.
            ('press_button_topic', '/elevator/press_button'),
            ('mission_cmd_topic', '/mission/cmd'),
            ('press_done_topic', '/elevator/press_done'),
            ('mission_start_topic', '/mission/start'),
            ('mission_loaded_topic', '/mission/loaded'),
            ('mission_unloaded_topic', '/mission/unloaded'),
            ('mission_enable_topic', '/mission/enable'),
            ('mission_restart_topic', '/mission/restart'),
            ('nt_press_button_key', 'Pi/elevator/press_button'),
            ('nt_mission_cmd_key', 'Pi/mission/cmd'),
            ('nt_press_done_key', 'Robot/elevator/press_done'),
            ('nt_mission_start_key', 'Robot/mission/start_pressed'),
            ('nt_mission_loaded_key', 'Robot/mission/loaded'),
            ('nt_mission_unloaded_key', 'Robot/mission/unloaded'),
            ('nt_mission_enable_key', 'Robot/mission/enable'),
            ('nt_mission_restart_key', 'Robot/mission/restart_pressed'),
            ('mission_in_rate_hz', 20.0),
            ('odom_publish_rate', 50.0),
            ('imu_publish_rate', 100.0),
            ('tag_publish_rate', 20.0),
            ('publish_odom_tf', True),
            ('base_frame_id', 'base_link'),
            ('odom_frame_id', 'odom'),
            ('imu_frame_id', 'imu_link'),
            ('tag_frame_id', 'limelight'),
            ('yaw_variance', 0.01),
            ('yaw_rate_variance', 0.001),
            ('accel_xy_variance', 0.1),
        ])
        gp = lambda n: self.get_parameter(n).value

        # ---- NT4 client ----
        self._nt = ntcore.NetworkTableInstance.getDefault()
        self._nt.startClient4(gp('nt_client_name'))
        server_addr = gp('server_address').strip()
        if server_addr:
            self._nt.setServer(server_addr, ntcore.NetworkTableInstance.kDefaultPort4)
            self.get_logger().info(f"NT4 server: {server_addr}:{ntcore.NetworkTableInstance.kDefaultPort4}")
        else:
            team = int(gp('team_number'))
            self._nt.setServerTeam(team)
            self.get_logger().info(
                f"NT4 server: team {team} (10.{team // 100}.{team % 100}.2)")

        # ---- ROS2 -> NT (cmd_vel) ----
        self._cmd_lock = Lock()
        self._last_cmd_stamp: Optional[float] = None
        self._heartbeat = 0
        if gp('enable_cmd_vel_out'):
            cmd_table = self._nt.getTable(gp('nt_cmd_table'))
            self._pub_vx = cmd_table.getDoubleTopic('vx').publish()
            self._pub_vy = cmd_table.getDoubleTopic('vy').publish()
            self._pub_omega = cmd_table.getDoubleTopic('omega').publish()
            self._pub_heartbeat = cmd_table.getIntegerTopic('heartbeat').publish()
            self._pub_ts = cmd_table.getDoubleTopic('timestamp').publish()
            self._publish_cmd(0.0, 0.0, 0.0)

            if gp('cmd_vel_stamped'):
                self.create_subscription(
                    TwistStamped, gp('cmd_vel_topic'),
                    self._on_cmd_vel_stamped, 10)
            else:
                self.create_subscription(
                    Twist, gp('cmd_vel_topic'),
                    self._on_cmd_vel, 10)

            self.create_timer(0.05, self._cmd_vel_watchdog)

        # ---- NT -> ROS2 (odom) ----
        if gp('enable_odom_in'):
            odom_table = self._nt.getTable(gp('nt_odom_table'))
            self._sub_odom_x = odom_table.getDoubleTopic('x').subscribe(0.0)
            self._sub_odom_y = odom_table.getDoubleTopic('y').subscribe(0.0)
            self._sub_odom_theta = odom_table.getDoubleTopic('theta').subscribe(0.0)
            self._sub_odom_vx = odom_table.getDoubleTopic('vx').subscribe(0.0)
            self._sub_odom_vy = odom_table.getDoubleTopic('vy').subscribe(0.0)
            self._sub_odom_omega = odom_table.getDoubleTopic('omega').subscribe(0.0)
            self._odom_pub = self.create_publisher(Odometry, gp('odom_topic'), 10)
            self._tf_broadcaster = (
                TransformBroadcaster(self) if gp('publish_odom_tf') else None)
            rate = float(gp('odom_publish_rate'))
            self.create_timer(1.0 / rate, self._publish_odom)

        # ---- NT -> ROS2 (imu) ----
        if gp('enable_imu_in'):
            imu_table = self._nt.getTable(gp('nt_imu_table'))
            self._sub_imu_yaw = imu_table.getDoubleTopic('yaw').subscribe(0.0)
            self._sub_imu_yaw_rate = imu_table.getDoubleTopic('yaw_rate').subscribe(0.0)
            self._sub_imu_accel_x = imu_table.getDoubleTopic('accel_x').subscribe(0.0)
            self._sub_imu_accel_y = imu_table.getDoubleTopic('accel_y').subscribe(0.0)
            self._imu_pub = self.create_publisher(Imu, gp('imu_topic'), 10)
            rate = float(gp('imu_publish_rate'))
            self.create_timer(1.0 / rate, self._publish_imu)

        # ---- NT -> ROS2 (limelight tag) ----
        if gp('enable_limelight_in'):
            ll_table = self._nt.getTable(gp('nt_limelight_table'))
            # tv: 1 if a target is currently visible, 0 otherwise
            self._sub_ll_tv = ll_table.getIntegerTopic('tv').subscribe(0)
            self._sub_ll_tid = ll_table.getIntegerTopic('tid').subscribe(-1)
            self._sub_ll_tx = ll_table.getDoubleTopic('tx').subscribe(0.0)
            self._sub_ll_ty = ll_table.getDoubleTopic('ty').subscribe(0.0)
            self._sub_ll_ta = ll_table.getDoubleTopic('ta').subscribe(0.0)
            # targetpose_robotspace: [x, y, z, pitch_deg, yaw_deg, roll_deg] in robot frame
            self._sub_ll_pose = ll_table.getDoubleArrayTopic(
                'targetpose_robotspace').subscribe([])
            self._tag_pub = self.create_publisher(
                TagDetection, gp('tag_topic'), 10)
            rate = float(gp('tag_publish_rate'))
            self.create_timer(1.0 / rate, self._publish_tag)

        # ---- mission IO (NT <-> ROS) ----
        if gp('enable_mission_io'):
            # Pi -> NT: press_button (ROS String forwarded as NT string).
            self._press_btn_pub = self._nt.getStringTopic(
                gp('nt_press_button_key')).publish()
            self._press_btn_pub.set('')   # clear any stale value on boot
            self.create_subscription(
                String, gp('press_button_topic'),
                self._on_press_button, 5)

            # Pi -> NT: mission_cmd. Separate string topic so the
            # lift+pusher mechanism can run in parallel with the
            # pneumatic-press path on the Rio side.
            self._mission_cmd_pub = self._nt.getStringTopic(
                gp('nt_mission_cmd_key')).publish()
            self._mission_cmd_pub.set('')
            self.create_subscription(
                String, gp('mission_cmd_topic'),
                self._on_mission_cmd, 5)

            # NT -> Pi: edge-triggered bools. We subscribe with a sentinel
            # of False so the first reading is well-defined; track the
            # previous value to detect rising edges.
            self._sub_press_done = self._nt.getBooleanTopic(
                gp('nt_press_done_key')).subscribe(False)
            self._sub_mission_start = self._nt.getBooleanTopic(
                gp('nt_mission_start_key')).subscribe(False)
            self._sub_mission_loaded = self._nt.getBooleanTopic(
                gp('nt_mission_loaded_key')).subscribe(False)
            self._sub_mission_unloaded = self._nt.getBooleanTopic(
                gp('nt_mission_unloaded_key')).subscribe(False)
            # Hold-to-run dead-man (level) + restart edge.
            self._sub_mission_enable = self._nt.getBooleanTopic(
                gp('nt_mission_enable_key')).subscribe(False)
            self._sub_mission_restart = self._nt.getBooleanTopic(
                gp('nt_mission_restart_key')).subscribe(False)

            self._prev_press_done = False
            self._prev_mission_start = False
            self._prev_mission_loaded = False
            self._prev_mission_unloaded = False
            self._prev_mission_enable = False
            self._prev_mission_restart = False

            self._pub_press_done = self.create_publisher(
                Bool, gp('press_done_topic'), 5)
            self._pub_mission_start = self.create_publisher(
                Bool, gp('mission_start_topic'), 5)
            self._pub_mission_loaded = self.create_publisher(
                Bool, gp('mission_loaded_topic'), 5)
            self._pub_mission_unloaded = self.create_publisher(
                Bool, gp('mission_unloaded_topic'), 5)
            # Latched QoS for the enable bool — supervisor needs to
            # know the current value when it starts up, not wait for
            # the next change.
            enable_qos = QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._pub_mission_enable = self.create_publisher(
                Bool, gp('mission_enable_topic'), enable_qos)
            # Seed the latched value so a supervisor that starts
            # before the operator has touched the button sees False.
            self._pub_mission_enable.publish(Bool(data=False))
            self._pub_mission_restart = self.create_publisher(
                Bool, gp('mission_restart_topic'), 5)

            rate = float(gp('mission_in_rate_hz'))
            self.create_timer(1.0 / rate, self._poll_mission_inputs)

        self.get_logger().info('nt_bridge ready')

    # ===== cmd_vel out =====
    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._cmd_lock:
            self._last_cmd_stamp = self.get_clock().now().nanoseconds * 1e-9
        self._publish_cmd(msg.linear.x, msg.linear.y, msg.angular.z)

    def _on_cmd_vel_stamped(self, msg: TwistStamped) -> None:
        with self._cmd_lock:
            self._last_cmd_stamp = self.get_clock().now().nanoseconds * 1e-9
        self._publish_cmd(msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)

    def _publish_cmd(self, vx: float, vy: float, omega: float) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self._pub_vx.set(float(vx))
        self._pub_vy.set(float(vy))
        self._pub_omega.set(float(omega))
        self._heartbeat += 1
        self._pub_heartbeat.set(self._heartbeat)
        self._pub_ts.set(now)

    def _cmd_vel_watchdog(self) -> None:
        max_age = float(self.get_parameter('cmd_vel_max_age').value)
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._cmd_lock:
            last = self._last_cmd_stamp
        if last is None or (now - last) > max_age:
            self._publish_cmd(0.0, 0.0, 0.0)

    # ===== odom in =====
    def _publish_odom(self) -> None:
        x = self._sub_odom_x.get()
        y = self._sub_odom_y.get()
        theta = self._sub_odom_theta.get()
        vx = self._sub_odom_vx.get()
        vy = self._sub_odom_vy.get()
        omega = self._sub_odom_omega.get()

        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(theta)
        odom_frame = self.get_parameter('odom_frame_id').value
        base_frame = self.get_parameter('base_frame_id').value

        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = odom_frame
        msg.child_frame_id = base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = omega
        self._odom_pub.publish(msg)

        if self._tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = odom_frame
            t.child_frame_id = base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(t)

    # ===== imu in =====
    def _publish_imu(self) -> None:
        yaw = self._sub_imu_yaw.get()
        yaw_rate = self._sub_imu_yaw_rate.get()
        ax = self._sub_imu_accel_x.get()
        ay = self._sub_imu_accel_y.get()

        qx, qy, qz, qw = yaw_to_quat(yaw)
        yaw_var = float(self.get_parameter('yaw_variance').value)
        yaw_rate_var = float(self.get_parameter('yaw_rate_variance').value)
        accel_var = float(self.get_parameter('accel_xy_variance').value)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('imu_frame_id').value
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        # roll/pitch unknown, yaw known
        msg.orientation_covariance = [
            LARGE_COV, 0.0, 0.0,
            0.0, LARGE_COV, 0.0,
            0.0, 0.0, yaw_var,
        ]
        msg.angular_velocity.z = yaw_rate
        msg.angular_velocity_covariance = [
            LARGE_COV, 0.0, 0.0,
            0.0, LARGE_COV, 0.0,
            0.0, 0.0, yaw_rate_var,
        ]
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration_covariance = [
            accel_var, 0.0, 0.0,
            0.0, accel_var, 0.0,
            0.0, 0.0, LARGE_COV,
        ]
        self._imu_pub.publish(msg)

    # ===== limelight tag in =====
    def _publish_tag(self) -> None:
        tv = int(self._sub_ll_tv.get())

        msg = TagDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('tag_frame_id').value

        if tv != 1:
            # No target visible — publish sentinel so consumers see liveness.
            msg.tag_id = -1
            msg.area = 0.0
            msg.tx_deg = 0.0
            msg.ty_deg = 0.0
            msg.has_pose = False
            self._tag_pub.publish(msg)
            return

        msg.tag_id = int(self._sub_ll_tid.get())
        msg.area = float(self._sub_ll_ta.get())
        msg.tx_deg = float(self._sub_ll_tx.get())
        msg.ty_deg = float(self._sub_ll_ty.get())

        pose = self._sub_ll_pose.get()
        if pose is not None and len(pose) >= 6:
            x, y, z, pitch_deg, yaw_deg, roll_deg = pose[:6]
            msg.has_pose = True
            msg.pose_in_robot_frame.position.x = float(x)
            msg.pose_in_robot_frame.position.y = float(y)
            msg.pose_in_robot_frame.position.z = float(z)
            qx, qy, qz, qw = rpy_deg_to_quat(roll_deg, pitch_deg, yaw_deg)
            msg.pose_in_robot_frame.orientation.x = qx
            msg.pose_in_robot_frame.orientation.y = qy
            msg.pose_in_robot_frame.orientation.z = qz
            msg.pose_in_robot_frame.orientation.w = qw
        else:
            msg.has_pose = False

        self._tag_pub.publish(msg)


    # ===== mission IO =====
    def _on_press_button(self, msg: String) -> None:
        # Forward whatever the supervisor says verbatim. RoboRIO is
        # responsible for picking the right pneumatic setpoint per
        # button name ('call_up' / 'floor_4' / ...).
        self._press_btn_pub.set(msg.data)
        self.get_logger().info(f'NT press_button -> {msg.data!r}')

    def _on_mission_cmd(self, msg: String) -> None:
        # Forward "load" | "unload" | "stow" to the Rio's lift+pusher
        # state machine. Validation lives on the Rio side; we forward
        # whatever the supervisor sends.
        self._mission_cmd_pub.set(msg.data)
        self.get_logger().info(f'NT mission_cmd -> {msg.data!r}')

    def _poll_mission_inputs(self) -> None:
        # Rising-edge detection: publish Bool(True) on False->True.
        # We do NOT publish Bool(False) on the falling edge — the
        # supervisor's latches only care about the event, not the
        # held state, so emitting False would just be noise.
        self._edge_publish('press_done', self._sub_press_done.get(),
                           '_prev_press_done', self._pub_press_done)
        self._edge_publish('mission_start', self._sub_mission_start.get(),
                           '_prev_mission_start', self._pub_mission_start)
        self._edge_publish('mission_loaded', self._sub_mission_loaded.get(),
                           '_prev_mission_loaded', self._pub_mission_loaded)
        self._edge_publish('mission_unloaded', self._sub_mission_unloaded.get(),
                           '_prev_mission_unloaded', self._pub_mission_unloaded)
        # Enable is level-triggered: republish on every change so the
        # supervisor sees the held state, not just events. Latched QoS
        # means late subscribers also pick up the current value.
        cur_enable = bool(self._sub_mission_enable.get())
        if cur_enable != self._prev_mission_enable:
            self._pub_mission_enable.publish(Bool(data=cur_enable))
            self.get_logger().info(f'NT mission_enable -> {cur_enable}')
            self._prev_mission_enable = cur_enable
        # Restart is edge-triggered like start/loaded/unloaded.
        self._edge_publish('mission_restart', self._sub_mission_restart.get(),
                           '_prev_mission_restart', self._pub_mission_restart)

    def _edge_publish(self, label: str, current: bool, prev_attr: str, pub) -> None:
        prev = getattr(self, prev_attr)
        if current and not prev:
            msg = Bool()
            msg.data = True
            pub.publish(msg)
            self.get_logger().info(f'NT rising edge: {label}')
        setattr(self, prev_attr, bool(current))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NtBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

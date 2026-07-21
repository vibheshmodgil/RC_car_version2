"""ROS2 bridge between the car's ROS2 graph and the existing, unmodified
DevKit REST/WS API + Pi sensor drivers.

Does NOT reimplement esp32_link/imu/lidar — it imports pi/esp32_link.py,
pi/imu.py, pi/lidar.py directly (the `pi_src_path` parameter, default
/home/pi/car/pi, is put on sys.path) so there is exactly one copy of that
logic, same as pi/webapp/hub.py and pi/main.py already share it.

Topics/services (see docs/hardware-architecture/v5-ros2-bridge.md):
  sub  cmd_vel        geometry_msgs/Twist   -> per-wheel PWM via /api/motor
  pub  imu/data       sensor_msgs/Imu       <- pi/imu.py (BNO055, NDOF)
  pub  scan           sensor_msgs/LaserScan <- pi/lidar.py (YDLIDAR X2)
  srv  estop          std_srvs/Trigger      -> POST /api/estop
  srv  estop_clear    std_srvs/Trigger      -> POST /api/estop/clear

Safety: wheels must already be armed via the existing dashboard/DevKit UI
— this node never arms them itself. A 0.1 s timer re-sends the latest
cmd_vel (inside the DEADMAN_RESEND_S=0.15 budget in pi/config.py) and
falls back to zero PWM if no cmd_vel has arrived in 0.5 s — a second,
ROS-side deadman layered on top of the DevKit's own 800 ms one.
"""
import math
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from sensor_msgs.msg import Imu, LaserScan
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster

# YDLIDAR X2 datasheet limits (meters) — confirm against the physical unit.
LIDAR_RANGE_MIN_M = 0.12
LIDAR_RANGE_MAX_M = 8.0
LIDAR_BINS = 360  # 1 deg resolution

CMD_VEL_STALE_S = 0.5


def euler_deg_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quaternion:
    """Roll/pitch/yaw in degrees (ZYX intrinsic) -> geometry_msgs/Quaternion.

    NOTE: BNO055 heading is a compass bearing (clockwise from north);
    ROS yaw is counter-clockwise from the X axis (REP 103, ENU). This
    passes yaw straight through — verify the sign on real hardware (spin
    the car and check RViz's IMU arrow turns the same way) and negate
    yaw_deg here if it turns backwards.
    """
    roll, pitch, yaw = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class CarBridge(Node):
    def __init__(self):
        super().__init__('car_bridge')

        self.declare_parameter('pi_src_path', '/home/pi/car/pi')
        self.declare_parameter('linear_scale', 255.0)
        self.declare_parameter('angular_scale', 150.0)

        pi_src_path = self.get_parameter('pi_src_path').value
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)

        # pi/*.py use bare `from config import ...` — same sys.path trick
        # pi/webapp/hub.py and pi/main.py rely on by running with cwd=pi/.
        if pi_src_path not in sys.path:
            sys.path.insert(0, pi_src_path)

        from esp32_link import Esp32Link  # noqa: E402  (deferred, needs sys.path above)
        self.link = Esp32Link()

        # ------------------------------------------------------- cmd_vel
        self._last_twist = Twist()
        self._last_twist_at = self.get_clock().now()
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_timer(0.1, self._on_drive_timer)

        # ------------------------------------------------------- e-stop
        self.create_service(Trigger, 'estop', self._on_estop)
        self.create_service(Trigger, 'estop_clear', self._on_estop_clear)

        # ------------------------------------------------------- IMU
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        try:
            from imu import Imu as ImuDriver
            self.imu = ImuDriver()
            self.create_timer(0.05, self._publish_imu)  # ~20 Hz
        except Exception as e:
            self.imu = None
            self.get_logger().warn(f'IMU unavailable, skipping imu/data: {e}')

        # ------------------------------------------------------- LiDAR
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self._latest_scan_pts = None
        try:
            from lidar import Lidar as LidarDriver
            self.lidar = LidarDriver()
            threading.Thread(target=self._lidar_loop, daemon=True).start()
            self.create_timer(0.1, self._publish_scan)  # ~10 Hz
        except Exception as e:
            self.lidar = None
            self.get_logger().warn(f'LiDAR unavailable, skipping scan: {e}')

        # ------------------------------------------------------- static TF
        # Zero-offset placeholders — replace with measured mounting offsets.
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._send_static_tf()

        self.get_logger().info('car_bridge up: cmd_vel, estop/estop_clear, imu/data, scan')

    # ----------------------------------------------------------- cmd_vel
    def _on_cmd_vel(self, msg: Twist):
        self._last_twist = msg
        self._last_twist_at = self.get_clock().now()

    def _on_drive_timer(self):
        age_s = (self.get_clock().now() - self._last_twist_at).nanoseconds / 1e9
        twist = self._last_twist if age_s <= CMD_VEL_STALE_S else Twist()

        left = twist.linear.x * self.linear_scale - twist.angular.z * self.angular_scale
        right = twist.linear.x * self.linear_scale + twist.angular.z * self.angular_scale
        left_pwm = max(-255, min(255, int(left)))
        right_pwm = max(-255, min(255, int(right)))

        try:
            self.link.motor_pwm('lf', left_pwm)
            self.link.motor_pwm('lr', left_pwm)
            self.link.motor_pwm('rf', right_pwm)
            self.link.motor_pwm('rr', right_pwm)
        except Exception as e:
            self.get_logger().warn(f'drive command failed: {e}')

    # ------------------------------------------------------------- e-stop
    def _on_estop(self, request, response):
        try:
            self.link.estop()
            response.success = True
            response.message = 'estop sent'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _on_estop_clear(self, request, response):
        try:
            self.link.estop_clear()
            response.success = True
            response.message = 'estop cleared'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # ----------------------------------------------------------------- IMU
    def _publish_imu(self):
        reading = self.imu.reading
        if not reading.get('ok'):
            return
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation = euler_deg_to_quaternion(reading['r'], reading['p'], reading['h'])
        # pi/imu.py doesn't parse raw gyro/accel registers (only fused
        # euler angles), so mark those two fields as unavailable per
        # REP 145 (element [0] = -1). Orientation covariance is left at
        # the default zero — no real estimate, but orientation itself is
        # valid, unlike the other two.
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(msg)

    # --------------------------------------------------------------- LiDAR
    def _lidar_loop(self):
        try:
            for pts in self.lidar.scans():
                self._latest_scan_pts = pts
        except Exception as e:
            self.get_logger().warn(f'lidar read loop stopped: {e}')

    def _publish_scan(self):
        pts = self._latest_scan_pts
        if not pts:
            return
        ranges = [float('inf')] * LIDAR_BINS
        for angle_deg, dist_mm in pts:
            if dist_mm <= 0:
                continue
            idx = int(angle_deg) % LIDAR_BINS
            ranges[idx] = dist_mm / 1000.0

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_link'
        msg.angle_min = 0.0
        msg.angle_increment = 2 * math.pi / LIDAR_BINS
        msg.angle_max = msg.angle_increment * (LIDAR_BINS - 1)
        msg.range_min = LIDAR_RANGE_MIN_M
        msg.range_max = LIDAR_RANGE_MAX_M
        msg.ranges = ranges
        self.scan_pub.publish(msg)

    # ------------------------------------------------------------- static TF
    def _send_static_tf(self):
        now = self.get_clock().now().to_msg()
        transforms = []
        for child in ('imu_link', 'laser_link'):
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'base_link'
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            transforms.append(t)
        self._tf_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = CarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

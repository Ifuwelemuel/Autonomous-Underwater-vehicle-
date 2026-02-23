import math

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String


class ImuDashboardAdapter(Node):
    """Converts raw IMU data into human-friendly topics for dashboards."""

    def __init__(self):
        super().__init__("imu_dashboard_adapter")

        self.declare_parameter("imu_topic", "imu/data_raw")
        self.declare_parameter("accel_stable_threshold_g", 0.05)
        self.declare_parameter("accel_move_threshold_g", 0.20)
        self.declare_parameter("turning_threshold_deg_s", 20.0)

        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.accel_stable_threshold_g = float(self.get_parameter("accel_stable_threshold_g").value)
        self.accel_move_threshold_g = float(self.get_parameter("accel_move_threshold_g").value)
        self.turning_threshold_deg_s = float(self.get_parameter("turning_threshold_deg_s").value)

        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 20)

        self.pub_accel_g = self.create_publisher(Vector3Stamped, "imu/dashboard/accel_g", 10)
        self.pub_gyro_deg_s = self.create_publisher(Vector3Stamped, "imu/dashboard/gyro_deg_s", 10)
        self.pub_tilt_deg = self.create_publisher(Vector3Stamped, "imu/dashboard/tilt_deg", 10)
        self.pub_accel_mag_g = self.create_publisher(Float32, "imu/dashboard/accel_magnitude_g", 10)
        self.pub_turn_rate_deg_s = self.create_publisher(Float32, "imu/dashboard/turn_rate_deg_s", 10)
        self.pub_state = self.create_publisher(String, "imu/dashboard/state", 10)

        self.get_logger().info(f"IMU dashboard adapter listening on '{self.imu_topic}'")

    def imu_callback(self, msg: Imu) -> None:
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        ax_g = ax / 9.80665
        ay_g = ay / 9.80665
        az_g = az / 9.80665
        gx_deg_s = math.degrees(gx)
        gy_deg_s = math.degrees(gy)
        gz_deg_s = math.degrees(gz)

        roll_deg = math.degrees(math.atan2(ay, az))
        pitch_deg = math.degrees(math.atan2(-ax, math.sqrt((ay * ay) + (az * az))))

        accel_mag_g = math.sqrt((ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g))
        turn_rate_deg_s = math.sqrt(
            (gx_deg_s * gx_deg_s) + (gy_deg_s * gy_deg_s) + (gz_deg_s * gz_deg_s)
        )

        accel_delta_g = abs(accel_mag_g - 1.0)
        motion_state = self.classify_motion(accel_delta_g, turn_rate_deg_s)

        accel_g_msg = Vector3Stamped()
        accel_g_msg.header = msg.header
        accel_g_msg.vector.x = ax_g
        accel_g_msg.vector.y = ay_g
        accel_g_msg.vector.z = az_g
        self.pub_accel_g.publish(accel_g_msg)

        gyro_deg_s_msg = Vector3Stamped()
        gyro_deg_s_msg.header = msg.header
        gyro_deg_s_msg.vector.x = gx_deg_s
        gyro_deg_s_msg.vector.y = gy_deg_s
        gyro_deg_s_msg.vector.z = gz_deg_s
        self.pub_gyro_deg_s.publish(gyro_deg_s_msg)

        tilt_deg_msg = Vector3Stamped()
        tilt_deg_msg.header = msg.header
        tilt_deg_msg.vector.x = roll_deg
        tilt_deg_msg.vector.y = pitch_deg
        tilt_deg_msg.vector.z = 0.0
        self.pub_tilt_deg.publish(tilt_deg_msg)

        accel_mag_msg = Float32()
        accel_mag_msg.data = float(accel_mag_g)
        self.pub_accel_mag_g.publish(accel_mag_msg)

        turn_rate_msg = Float32()
        turn_rate_msg.data = float(turn_rate_deg_s)
        self.pub_turn_rate_deg_s.publish(turn_rate_msg)

        state_msg = String()
        state_msg.data = motion_state
        self.pub_state.publish(state_msg)

    def classify_motion(self, accel_delta_g: float, turn_rate_deg_s: float) -> str:
        if turn_rate_deg_s >= self.turning_threshold_deg_s:
            return "TURNING"
        if accel_delta_g >= self.accel_move_threshold_g:
            return "ACCELERATING"
        if accel_delta_g <= self.accel_stable_threshold_g:
            return "STABLE"
        return "MOVING"


def main(args=None):
    rclpy.init(args=args)
    node = ImuDashboardAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

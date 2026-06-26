#!/usr/bin/env python3
"""
Xbox controller node for servo actuation via MAVROS RC override.

Reads joystick axes from /joy topic and publishes RC override
commands to /mavros/rc/override at 10 Hz.

Parameters:
    axis_thruster (int): Joystick axis index for thruster (default: 5)
    axis_yaw (int): Joystick axis index for yaw (default: 3)
    axis_pitch (int): Joystick axis index for pitch (default: 1)

RC Channel Mapping:
    Channel 2 → Pitch (SERVO2, elevator)
    Channel 3 → Throttle / Thruster (SERVO output)
    Channel 4 → Yaw (SERVO3, rudder)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mavros_msgs.msg import OverrideRCIn

# PWM limits
PWM_MIN = 900
PWM_CENTER = 1500
PWM_MAX = 2000
NO_OVERRIDE = 0


class xbox_node(Node):
    def __init__(self):
        super().__init__('xbox_node')

        # Declare parameters
        self.declare_parameter('axis_thruster', 5)
        self.declare_parameter('axis_yaw', 3)
        self.declare_parameter('axis_pitch', 1)

        # PWM range parameters
        self.declare_parameter('pwm_min', PWM_MIN)
        self.declare_parameter('pwm_max', PWM_MAX)
        self.declare_parameter('pwm_center', PWM_CENTER)

        # Deadzone for joystick input (ignore small values)
        self.declare_parameter('deadzone', 0.05)

        # Publishing rate in Hz
        self.declare_parameter('publish_rate', 10.0)

        # Read parameters
        self.axis_thruster = self.get_parameter('axis_thruster').value
        self.axis_yaw = self.get_parameter('axis_yaw').value
        self.axis_pitch = self.get_parameter('axis_pitch').value
        self.pwm_min = self.get_parameter('pwm_min').value
        self.pwm_max = self.get_parameter('pwm_max').value
        self.pwm_center = self.get_parameter('pwm_center').value
        self.deadzone = self.get_parameter('deadzone').value
        publish_rate = self.get_parameter('publish_rate').value

        # Current axis values (updated by joy callback)
        self.pitch_val = 0.0
        self.yaw_val = 0.0
        self.thruster_val = 0.0

        # Track if we have received any joy messages
        self.joy_received = False

        # Subscribe to /joy topic (from joy_node / xbox driver)
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Publisher for RC override
        self.rc_pub = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            10
        )

        # Timer to publish RC override at a steady rate
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_override)


        self.get_logger().info(
            f'Xbox servo node started\n'
            f'  Axis mapping: pitch={self.axis_pitch}, '
            f'yaw={self.axis_yaw}, thruster={self.axis_thruster}\n'
            f'  PWM range: {self.pwm_min}-{self.pwm_max}, '
            f'center={self.pwm_center}\n'
            f'  Publishing at {publish_rate} Hz'
        )

    def joy_callback(self, msg: Joy):
        """Read joystick axis values from the Xbox controller."""
        if not self.joy_received:
            self.get_logger().info('Joystick connected — receiving data')
            self.joy_received = True

        axes = msg.axes
        num_axes = len(axes)

        if self.axis_pitch < num_axes:
            self.pitch_val = axes[self.axis_pitch]

        if self.axis_yaw < num_axes:
            self.yaw_val = axes[self.axis_yaw]

        if self.axis_thruster < num_axes:
            self.thruster_val = axes[self.axis_thruster]

    def apply_deadzone(self, value: float) -> float:
        """Zero out values below the deadzone threshold."""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def axis_to_pwm(self, value: float) -> int:
        """
        Convert a joystick axis value (-1.0 to 1.0) to PWM (1000-2000).

        Axis center (0.0) maps to PWM_CENTER (1500).
        Axis -1.0 maps to PWM_MIN (1000).
        Axis +1.0 maps to PWM_MAX (2000).
        """
        value = self.apply_deadzone(value)
        value = max(-1.0, min(1.0, value))
        pwm = self.pwm_center + int(value * (self.pwm_max - self.pwm_min) / 2)
        return max(self.pwm_min, min(self.pwm_max, pwm))

    def trigger_to_pwm(self, value: float) -> int:
        """
        Convert a trigger axis to PWM.

        Xbox triggers typically output 1.0 (released) to -1.0 (fully pressed).
        Maps: 1.0 (released) → PWM_MIN, -1.0 (pressed) → PWM_MAX.
        """
        value = max(-1.0, min(1.0, value))
        # Remap from [1.0, -1.0] to [0.0, 1.0]
        normalized = (1.0 - value) / 2.0
        pwm = self.pwm_min + int(normalized * (self.pwm_max - self.pwm_min))
        return max(self.pwm_min, min(self.pwm_max, pwm))

    def publish_override(self):
        """Publish RC override message at the timer rate."""
        if not self.joy_received:
            return

        msg = OverrideRCIn()

        # Build the 18-channel array
        # All channels default to NO_OVERRIDE (0)
        channels = [1500] * 8 + [NO_OVERRIDE] * 10

        # RC channel 2 (index 1) → Pitch
        channels[1] = self.axis_to_pwm(self.pitch_val)

        # RC channel 3 (index 2) → Thruster
        channels[2] = self.trigger_to_pwm(self.thruster_val)

        # RC channel 4 (index 3) → Yaw
        channels[3] = self.axis_to_pwm(self.yaw_val)

        msg.channels = channels
        self.rc_pub.publish(msg)

    def destroy_node(self):
        """Release all overrides on shutdown."""
        self.get_logger().info('Shutting down — releasing RC overrides')
        msg = OverrideRCIn()
        msg.channels = [NO_OVERRIDE] * 18
        self.rc_pub.publish(msg)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = xbox_node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
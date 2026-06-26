#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mavros_msgs.msg import OverrideRCIn

class XboxNode(Node):
    def __init__(self):
        super().__init__('xbox_node')

        # --- Parameters ---
        self.declare_parameter('axis_yaw', 3)   # Usually Right Stick X
        self.declare_parameter('axis_pitch', 1) # Usually Left Stick Y
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('send_rate_hz', 20.0)

        self.axis_pitch_idx = int(self.get_parameter('axis_pitch').value)
        self.axis_yaw_idx   = int(self.get_parameter('axis_yaw').value)
        self.deadzone       = float(self.get_parameter('deadzone').value)
        send_rate           = float(self.get_parameter('send_rate_hz').value)

        # --- State ---
        # Storing normalized values (-1.0 to 1.0)
        self.pitch_input = 0.0   
        self.yaw_input = 0.0     

        # --- Publisher ---
        # Using rc/override to directly inject PWM to the servo rails
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # --- Subscriber ---
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # --- Timer ---
        self.create_timer(1.0 / send_rate, self.publish_control)

        self.get_logger().info('xbox_node ready — publishing PWM to /mavros/rc/override')

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def joy_callback(self, msg):
        # Read axes and apply deadzone
        self.pitch_input = self.apply_deadzone(msg.axes[self.axis_pitch_idx])
        self.yaw_input   = self.apply_deadzone(msg.axes[self.axis_yaw_idx])

        self.get_logger().info(
            f'Raw Pitch: {self.pitch_input:.2f} | Raw Yaw: {self.yaw_input:.2f}',
            throttle_duration_sec=0.5)

    def publish_control(self):
        msg = OverrideRCIn()
        
        # MAVROS RC Override requires an array of 18 channels. 
        # A value of '0' means "ignore / release control to ArduSub".
        # Values between 1100 and 1900 represent active PWM override.
        msg.channels = [0] * 18 

        # Map the -1.0 to 1.0 joystick range to standard 1100-1900 PWM
        # Center is 1500. Multiplier is 400 (1500 +/- 400 = 1100 to 1900)
        pitch_pwm = int(1500 + (self.pitch_input * 400))
        yaw_pwm   = int(1500 + (self.yaw_input * 400))

        # Channel mapping:
        # Index 0 = MAIN 1
        # Index 1 = MAIN 2 (Your Pitch Servo)
        # Index 2 = MAIN 3 (Your Yaw Servo)
        msg.channels[2] = pitch_pwm
        msg.channels[3] = yaw_pwm

        self.rc_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = XboxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

class AuvTeleopXbox(Node):
    def __init__(self):
        super().__init__('teleop_xbox_node')
        
        # 1. DECLARE PARAMETERS
        # We tell the node to expect these variables from the YAML file.
        # The second argument is the default value if the YAML file is missing.
        self.declare_parameter('axis_thruster', 1)
        self.declare_parameter('axis_yaw', 3)
        self.declare_parameter('axis_pitch', 4)
        self.declare_parameter('button_pump_fill', 5)
        self.declare_parameter('button_pump_empty', 4)
        self.declare_parameter('scale_thruster', 1.0)
        self.declare_parameter('scale_yaw', 1.0)
        self.declare_parameter('scale_pitch', 1.0)
        self.declare_parameter('deadzone', 0.1)

        # 2. GET PARAMETERS
        # We extract the values and store them as class variables to use later.
        self.axis_thruster = self.get_parameter('axis_thruster').value
        self.axis_yaw = self.get_parameter('axis_yaw').value
        self.axis_pitch = self.get_parameter('axis_pitch').value
        
        self.btn_fill = self.get_parameter('button_pump_fill').value
        self.btn_empty = self.get_parameter('button_pump_empty').value
        
        self.scale_thruster = self.get_parameter('scale_thruster').value
        self.scale_yaw = self.get_parameter('scale_yaw').value
        self.scale_pitch = self.get_parameter('scale_pitch').value
        self.deadzone = self.get_parameter('deadzone').value

        # 3. SET UP PUBLISHERS AND SUBSCRIBERS
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pump_pub = self.create_publisher(Int8, '/pump_cmd', 10)

        self.get_logger().info("Xbox Teleop Node configured and running.")

    def joy_callback(self, msg):
        # --- PROPORTIONAL CONTROL (Cmd_vel for Thruster & Servos) ---
        twist = Twist()
        
        # Read the raw axis values using the indices we loaded from the YAML
        raw_thruster = msg.axes[self.axis_thruster]
        raw_yaw = msg.axes[self.axis_yaw]
        raw_pitch = msg.axes[self.axis_pitch]

        # Apply deadzone to prevent drift
        if abs(raw_thruster) < self.deadzone: raw_thruster = 0.0
        if abs(raw_yaw) < self.deadzone: raw_yaw = 0.0
        if abs(raw_pitch) < self.deadzone: raw_pitch = 0.0

        # Apply scaling and assign to Twist message
        # x is forward/backward, z is left/right steering, y is up/down pitch
        twist.linear.x = raw_thruster * self.scale_thruster
        twist.angular.z = raw_yaw * self.scale_yaw
        twist.angular.y = raw_pitch * self.scale_pitch

        self.cmd_vel_pub.publish(twist)

        # --- BOOLEAN CONTROL (Pumps) ---
        pump_msg = Int8()
        
        # Read the button states (0 is unpressed, 1 is pressed)
        fill_pressed = msg.buttons[self.btn_fill]
        empty_pressed = msg.buttons[self.btn_empty]

        # Determine pump state: 1 = Fill, -1 = Empty, 0 = Off
        if fill_pressed == 1 and empty_pressed == 0:
            pump_msg.data = 1
        elif empty_pressed == 1 and fill_pressed == 0:
            pump_msg.data = -1
        else:
            pump_msg.data = 0 # Off if neither or both are pressed

        self.pump_pub.publish(pump_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AuvTeleopXbox()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
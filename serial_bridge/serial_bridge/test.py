import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class ArduinoSerialBridge(Node):
    def __init__(self):
        super().__init__('arduino_serial_bridge')
        
        # --- Serial Connection ---
        self.serial_port = None
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Successfully connected to Arduino.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
        
        # --- State Variables ---
        self.target_pitch = 0.0  
        self.target_yaw = 0.0   
        self.manual_throttle = 1500 
        self.ballast_request = 0  
        
        # --- Subscribers ---
        self.create_subscription(Float32MultiArray, '/servo_angles', self.servo_callback, 10)
        self.create_subscription(Float32MultiArray, '/thruster_command', self.thruster_callback, 10)
        
        # --- Timer ---
        # 0.1 seconds = 10 Hz loop rate. Adjust this if you need faster/slower hardware response.
        self.timer = self.create_timer(0.1, self.send_to_arduino)
        
        self.get_logger().info("Serial Bridge listening for topics and sending at 10Hz...")

    def servo_callback(self, msg):
        """Updates pitch and yaw internally. Does NOT send to Arduino."""
        if len(msg.data) >= 2:
            self.target_pitch = msg.data[0]
            self.target_yaw = msg.data[1]
            
    def thruster_callback(self, msg):
        """Updates throttle internally. Does NOT send to Arduino."""
        if len(msg.data) >= 1:
            self.manual_throttle = int(msg.data[0])

    def send_to_arduino(self):
        """Fires at a fixed rate to send the latest known state to the Arduino."""
        if self.serial_port is None or not self.serial_port.is_open:
            return

        command_string = f"C {self.target_pitch:.2f},{self.target_yaw:.2f},{self.manual_throttle},{self.ballast_request}\n"
        
        try:
            # We don't reset the input buffer here constantly so we can catch full Arduino replies
            self.serial_port.write(command_string.encode('ascii'))
            
            # Read any available response non-blockingly (or up to timeout)
            if self.serial_port.in_waiting > 0:
                response = self.serial_port.readline().decode('ascii', errors='ignore').strip()
                if response:
                    self.get_logger().info(f"Serial RX <- {response}")
                
        except serial.SerialTimeoutException:
            self.get_logger().warn("Serial write timed out! Arduino is too slow or disconnected.")
            self.serial_port.reset_output_buffer()
            
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    bridge_node = ArduinoSerialBridge()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safely close the serial port when shutting down
        if hasattr(bridge_node, 'serial_port') and bridge_node.serial_port is not None and bridge_node.serial_port.is_open:
            bridge_node.serial_port.close()
        
        bridge_node.destroy_node()
        
        # Prevent the double-shutdown crash at the end of your log
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
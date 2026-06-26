import rclpy 
from rclpy.node import node
import math
import os, json
from math import degrees
from sensor_msgs.msg import NavSatFix, Imu

#TODO visualize all of the data in some program? But I don't have a connection anyway :(

class Mission:
    def __init__(self):
        # Define the max mission distance, used for checking the mission before doing it
        self.max_mission_distance = 20000
        self.current_wp = None
        self.current_wp_lat = None
        self.current_wp_lon = None
        self.current_wp_depth = None
        self.mission_data = None

    # Checks if the mission is viable:
    def mission_viable(self):
        #TODO: get the package installed in the share directory:
        # package_share = get_package_share_directory("auv_navigation")
        # file_path = os.path.join(package_share, "config", "file.json")

        file_path = "/home/cpudrone/CPL/AUV_ws/src/auv_navigation/auv_navigation/config/mission.json"

        with open(file_path, "r") as file:
            self.mission_data = json.load(file)

        # Checks if the mission distance matches up
        if self.total_distance_left(1) < self.max_mission_distance:
            self.current_wp = 1
            self.update_target()
            return True, self.total_distance_left(1)
        else:
            return False, None

    # Calculates the total distance the AUV needs to swim in order to complete the mission
    # Takes the starting point as an argument, the argument is the waypoint index from which the function... 
    # If the function starts from waypoint number one, it will calculate the entire length of the path
    # Takes the starting point as an argument, the argument is the waypoint index from which the function should start counting
    # If the function starts from waypoint number one, it will calculate the entire length of the path
    def total_distance_left(self, starting_point):
        previous_lat = None
        previous_lon = None
        total_distance = 0

        for (waypoint_number, coordinates) in self.mission_data.items():
            if int(waypoint_number) >= starting_point:

                current_lat = coordinates['latitude']
                current_lon = coordinates['longitude']

                if previous_lat and previous_lon:
                    _, distance_between_waypoints = self.calculate_distance_and_bearing(previous_lon, previous_lat, current_lon, current_lat)
                    total_distance += distance_between_waypoints
                
                previous_lat = coordinates['latitude']
                previous_lon = coordinates['longitude']

        return total_distance

    def calculate_distance_and_bearing(self, lon1, lat1, lon2, lat2):
        geod = Geod(ellps="WGS84")
        fwd_azimuth, _, distance = geod.inv(lon1, lat1, lon2, lat2)
        bearing = fwd_azimuth

        # bearing = (fwd_azimuth+360) % 360

        return bearing, distance

    # Updates all of the target variables for the next waypoint. Returns False if there are no more waypoints
    def update_target(self):
        self.current_wp += 1
        if str(self.current_wp) in self.mission_data:
            self.current_wp_lat = float(self.mission_data[str(self.current_wp)]["latitude"])
            self.current_wp_lon = float(self.mission_data[str(self.current_wp)]["longitude"])
            self.current_wp_depth = float(self.mission_data[str(self.current_wp)]["depth"])
            return True
        else:
            return False          
            
            
            

        
        
        
class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner') # This is the name of the node

        # GPS buoy attributes
        self.buoy_gps_lock = 0
        self.buoy_latitude = 50.36
        self.buoy_longitude = 18.59
        
        #set max pitch and yaw 
        self.max_pitch = 40
        self.max_yaw = 40
        
        # AUV internal attributes
        self.current_heading = 0.0
        self.current_depth =0.0
        self.acceptance_radius = 2.0 # How close (in meters) to consider a waypoint "reached"

        # 1. Initialize the Mission
        self.active_mission = Mission()
        self.get_logger().info("Checking if mission is viable...")
        
        is_viable, total_dist = self.active_mission.mission_viable()

        if not is_viable:
            self.get_logger().error("Mission aborted: Path is too long or invalid.")
            return # Stops the node from doing anything else
            
        self.get_logger().info(f"Mission is a GO! Total distance: {total_dist:.2f} meters.")

        # 2. Setup Subscribers (Listening to the AUV)
        # Note: You may need to adjust the attribute names (msg.latitude, msg.yaw, etc.) 
        self.subscription = self.create_subscription(NavSatFix,'/mavros/global_position/global',self.gps_callback,10)
        
        self.heading_sub = self.create_subscription(Twiststamp, '/mavros/imu/data',self.heading_callback,10)
        
        #self.sub_depth = self.create_subscription(Twiststamp, 'emty',self.depth_callback,10)

        # 3. Setup Publishers (Giving orders)
        # TODO: Add your publisher here to send commands to the thrusters/controllers.
        # Example: self.command_pub = self.create_publisher(Twist, '/auv/cmd_vel', 10)

        # 4. Setup the Control Loop (The Heartbeat)
        timer_period = 0.5 # Runs twice a second (2Hz)
        self.timer = self.create_timer(timer_period, self.navigation_loop)
        

    # --- Callbacks for incoming data ---
    def gps_callback(self, msg):
        self.buoy_latitude = msg.latitude
        self.buoy_longitude = msg.longitude
        self.buoy_gps_lock = msg.status # Adjust based on your msg definition

    def heading_callback(self, msg):
        
        siny_cosp = 2 * (msg.w * msg.z + msg.x * msg.y)
        cosy_cosp = 1 - 2 * (msg.y * msg.y + msg.z * msg.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)
        self.current_heading = yaw_deg # Adjust based on your msg definition
      """  
    def depth_callback(self, msg):
        
        self.current_depth = msd.depth
        """


    # --- The Main Brain ---
    def navigation_loop(self):
        # Safety Check
        if self.buoy_gps_lock == 0:
            self.get_logger().warn("Waiting for GPS lock...", throttle_duration_sec=5.0)
            # TODO: Publish a "stop" command here so the AUV doesn't drift blind
            return

        # Grab the active target coordinates from the Mission object
        target_lat = self.active_mission.current_wp_lat
        target_lon = self.active_mission.current_wp_lon
        target_dep = self.active_mission.current_wp_depth
        
        if target_lat is None or target_lon is None:
            return # Failsafe just in case coordinates are empty

        # Calculate distance and bearing/yaw to the current waypoint
        target_bearing, distance_to_target = self.active_mission.calculate_distance_and_bearing(
            self.buoy_longitude, 
            self.buoy_latitude, 
            target_lon, 
            target_lat)
        
        
        # Calculate the desired pitch depth
        delta_depth  = target_dep - self.current_depth
        pitch_rad = math.atan2(delta_depth,distance_to_target) 
        
        #convert pitch to degrees 
        target_pitch = math.degrees(pitch_rad)
          
        # Check if we've arrived!
        if distance_to_target <= self.acceptance_radius:
            self.get_logger().info(f"Waypoint {self.active_mission.current_wp} reached!")
            
            # Load the next waypoint
            has_more_waypoints = self.active_mission.update_target()
            
            if not has_more_waypoints:
                self.get_logger().info("Final waypoint reached. Mission Complete!")
                # TODO: Publish a final "Stop" or "Surface" command to the AUV here
                self.timer.cancel() # Stop the navigation loop
        else:
            # Still traveling to the waypoint
            self.get_logger().info(
                f"Heading to WP {self.active_mission.current_wp}. "
                f"Dist: {distance_to_target:.1f}m, Target Bearing: {target_bearing:.1f}°"
                f"Dist: {distance_to_target:.1f}m, Target pitch: {target_pitch:.1f}", 
                throttle_duration_sec=2.0)
            
            
            


def main(args=None):
    rclpy.init(args=args)
    path_planner_node = PathPlanner()
    
    try:
        rclpy.spin(path_planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        path_planner_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
                
            
        
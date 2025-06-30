from pymavlink import mavutil
import time
import math

def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate horizontal distance between two GPS points
    Returns distance in meters
    """
    R = 6371000  # Earth radius in meters
    φ1 = math.radians(lat1)
    φ2 = math.radians(lat2)
    Δφ = math.radians(lat2 - lat1)
    Δλ = math.radians(lon2 - lon1)

    a = math.sin(Δφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(Δλ/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c
    
def get_gps_data():
    try:
        # Establish connection
        master = mavutil.mavlink_connection('/dev/ttyTHS0', baud=921600)
        #master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
        print("Waiting for heartbeat")
        master.wait_heartbeat()
        print("Heartbeat received")
    
        # Main loop
        while True:
            # Get both raw and processed GPS data
            msg_raw = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=3)
             
            if msg_raw:
                # Convert scaled integers to decimal degrees
                lat = msg_raw.lat / 1e7
                lon = msg_raw.lon / 1e7
                alt = msg_raw.alt / 1000  # Meters
                satellites = msg_raw.satellites_visible

                return lat, lon, alt
    
    except KeyboardInterrupt:
        print("\nExiting GPS monitor...")
    except Exception as e:
        print(f"Error: {str(e)}")

def start_mission():
    home_waypoint_lat = 34.4040307
    home_waypoint_lon = -119.6961773
    home_waypoint_alt = 0.17
    
    waypoint_lat = 34.4040435  # Replace with your target latitude
    waypoint_lon = -119.6961586   # Replace with your target longitude
    waypoint_alt = 0.01        # Altitude in meters (relative to home)
    
    # --- Connect to the vehicle ---
    #master = mavutil.mavlink_connection('/dev/ttyTHS0', baud=921600)
    master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
    print("Waiting for heartbeat")
    master.wait_heartbeat()
    print("Heartbeat received")
    
    # --- Set mode to GUIDED to allow mission upload ---
    master.set_mode_apm('GUIDED')
    time.sleep(2)
    
    # --- Clear existing mission ---
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    time.sleep(1)
    
    # --- Define mission items ---
    # Home/Takeoff point (required as first mission item)
    home_seq = 0
    home_cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    home_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    home_current = 1
    home_autocontinue = 1
    home_param = [0, 0, 0, 0, home_waypoint_lat, home_waypoint_lon, home_waypoint_alt]
    
    # Waypoint
    wp_seq = 1
    wp_cmd = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    wp_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    wp_current = 0
    wp_autocontinue = 1
    wp_param = [0, 0, 0, 0, waypoint_lat, waypoint_lon, waypoint_alt]
    
    # --- Send mission count ---
    master.mav.mission_count_send(master.target_system, master.target_component, 2)
    time.sleep(0.5)
    
    # --- Send mission items ---
    # Send takeoff/home
    master.mav.mission_item_send(
        master.target_system, master.target_component,
        home_seq, home_frame, home_cmd, home_current, home_autocontinue,
        *home_param
    )
    time.sleep(0.5)
    
    # Send waypoint
    master.mav.mission_item_send(
        master.target_system, master.target_component,
        wp_seq, wp_frame, wp_cmd, wp_current, wp_autocontinue,
        *wp_param
    )
    time.sleep(0.5)
    
    # --- Wait for mission ACK ---
    while True:
        msg = master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=5)
        if msg:
            print(f"Mission ACK received: type={msg.type}")
            break
    
    # --- Arm vehicle ---
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Vehicle armed!")
    
    # --- Set mode to AUTO to start mission ---
    master.set_mode_apm('AUTO')
    print("Mission started in AUTO mode.")
    
    # --- Monitor mission progress ---
    TARGET_RADIUS = 1.0  # 5 meter acceptance radius
    STOP_SPEED_THRESHOLD = 0.1  # m/s (adjust based on vehicle characteristics)
    TIMEOUT = 20  # 5 minute timeout
    start_time = time.time()

    while True:
        # Get current position and speed
        current_lat, current_lon, current_alt = get_gps_data()
        
        # Get vehicle speed (using MAVLink VFR_HUD message)
        msg_speed = master.recv_match(type='VFR_HUD', blocking=True, timeout=1)
        current_speed = msg_speed.groundspeed if msg_speed else 0.0
        
        # Calculate distances
        horiz_distance = haversine(current_lat, current_lon, 
                                waypoint_lat, waypoint_lon)
        alt_diff = abs(current_alt - waypoint_alt)
        
        
        print(f"Distance: {horiz_distance:.2f}m | Alt diff: {alt_diff:.2f}m | Speed: {current_speed:.2f}m/s")
        # Check arrival conditions
        if current_speed <= STOP_SPEED_THRESHOLD:
            if horiz_distance <= TARGET_RADIUS and alt_diff <= 2.0:
                print("Vehicle stopped within target area")
                break
        
        # Timeout check
        if time.time() - start_time > TIMEOUT:
            print("Mission timeout!")
            break
        
    time.sleep(1)  # Adjust polling frequency as needed
def set_servo_pwm_low():
    """Set servo PWM value using MAV_CMD_DO_SET_SERVO"""

    #master = mavutil.mavlink_connection('/dev/ttyTHS0', baud=921600)
    master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
    print("Waiting for heartbeat")
    master.wait_heartbeat()
    print("Heartbeat received")

    # Request AUTOPILOT_VERSION to test connection
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,  # confirmation
        0, 0, 0, 0, 0, 0, 0  # param1=0 (AUTOPILOT_VERSION), rest = 0
    )

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,  # Confirmation
        9,  #Servo Port in PWM Board
        1100,  # PWM value for demagnitized
        0,0,0,0,0   # Unused parameters
    )

def set_servo_pwm_high():
    """Set servo PWM value using MAV_CMD_DO_SET_SERVO"""

    #master = mavutil.mavlink_connection('/dev/ttyTHS0', baud=921600)
    master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
    print("Waiting for heartbeat")
    master.wait_heartbeat()
    print("Heartbeat received")

    # Request AUTOPILOT_VERSION to test connection
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,  # confirmation
        0, 0, 0, 0, 0, 0, 0  # param1=0 (AUTOPILOT_VERSION), rest = 0
    )

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,  # Confirmation
        9,  #Servo Port in PWM Board
        1900,  # PWM value for demagnitized
        0,0,0,0,0   # Unused parameters
    )


# Prepare velocity command (move forward in X axis, NED frame)
def send_forward_velocity(velocity=1.0, duration=1.0):
    """
    Move forward at 'velocity' m/s for 'duration' seconds.
    """
    #master = mavutil.mavlink_connection('/dev/ttyTHS0', baud=921600)
    master = mavutil.mavlink_connection('udp:0.0.0.0:14550')

    # Wait for heartbeat
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print("Heartbeat received")

    # Set mode to GUIDED (required for movement commands)
    master.set_mode('GUIDED')
    time.sleep(1)
    # Type mask: ignore position, acceleration, yaw
    type_mask = 0b0000111111000111
    frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

    # Send command at 10Hz for the duration
    start_time = time.time()
    while time.time() - start_time < duration:
        master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            master.target_system,
            master.target_component,
            frame,
            type_mask,
            0, 0, 0,         # x, y, z positions (not used)
            velocity, 0, 0,  # x, y, z velocity (m/s) - forward in X
            0, 0, 0,         # x, y, z acceleration (not used)
            0, 0             # yaw, yaw_rate (not used)
        )
        time.sleep(0.1)

    # Stop the vehicle after movement
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        frame,
        type_mask,
        0, 0, 0,
        0, 0, 0,   # Stop (zero velocity)
        0, 0, 0,
        0, 0
    )
    
    
if __name__ == "__main__":
    #print("MAIN LOG: ATTACHING")
    #time.sleep(2)
    #set_servo_pwm_high()
    #time.sleep(5)
    #print("MAIN LOG: Attaching Complete")

    print("MAIN LOG: Getting Home Location")
    home_lat, home_lon, home_alt = get_gps_data()
    print("Lat: " + str(home_lat) + ",Lon: " + str(home_lon) + ", Alt: " + str(home_alt))

    print("MAIN LOG: STARTING MISSION")
    start_mission()
    print("MAIN LOG: MISSION COMPLETE")

    time.sleep(40)

    print("MAIN LOG: Starting Drop Payload")
    set_servo_pwm_low()
    print("MAIN LOG: Dropped Payload")





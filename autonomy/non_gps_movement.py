from pymavlink import mavutil
import time
import numpy as np

# MAVLink connection
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=921600)
master.wait_heartbeat()

def set_origin_manual(lat, lon, alt):
    """Set origin manually when no GPS available"""
    master.mav.set_gps_global_origin_send(
        master.target_system,
        int(lat * 1e7),  # Latitude in 1E7 degrees
        int(lon * 1e7),  # Longitude in 1E7 degrees
        int(alt * 1000)  # Altitude in mm
    )
    print(f"Origin set to: {lat}, {lon}, {alt}")

def send_vision_position(x, y, z, roll, pitch, yaw):
    """Send vision-based position estimate"""
    current_time = int(time.time() * 1000000)  # microseconds
    
    master.mav.vision_position_estimate_send(
        current_time,  # timestamp
        x, y, z,       # position in meters (NED frame)
        roll, pitch, yaw  # orientation in radians
    )

def enable_non_gps_mode():
    """Configure ArduPilot for non-GPS operation"""
    # Set parameters for non-GPS navigation
    params = {
        'AHRS_EKF_TYPE': 3,      # Use EKF3
        'EK3_GPS_TYPE': 0,       # No GPS
        'EK3_POSNE_M_NSE': 0.1,  # Position noise
        'EK3_VELD_M_NSE': 0.1,   # Velocity noise
        'EK3_VELNE_M_NSE': 0.1,  # Velocity noise
    }
    
    for param, value in params.items():
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            param.encode('utf-8'),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        print(f"Set {param} = {value}")
        time.sleep(0.1)

def send_movement_command(vx, vy, yaw_rate):
    """Send movement command without GPS"""
    master.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,  # Velocity control
        0, 0, 0,  # position (ignored)
        vx, vy, 0,  # velocity
        0, 0, 0,  # acceleration (ignored)
        0, yaw_rate  # yaw, yaw_rate
    )

# Initialize non-GPS operation
print("Setting up non-GPS navigation...")

# Set a known origin point (your starting location)
# Replace with your actual coordinates
set_origin_manual(34.4208, -119.6982, 100)  # Santa Barbara coordinates

# Configure for non-GPS mode
enable_non_gps_mode()

# Set GUIDED mode
master.set_mode('GUIDED')
time.sleep(2)

# Arm the vehicle
master.arducopter_arm()
print("Vehicle armed in non-GPS mode")

# Example movement without GPS
try:
    while True:
        # Send position updates from your ArUco/vision system
        # These would come from your camera-based localization
        x, y, z = 0, 0, 0  # Replace with actual position from vision
        roll, pitch, yaw = 0, 0, 0  # Replace with actual orientation
        
        send_vision_position(x, y, z, roll, pitch, yaw)
        
        # Send movement commands
        send_movement_command(0.5, 0, 0)  # Move forward at 0.5 m/s
        
        time.sleep(0.25)  # 4Hz update rate
        
except KeyboardInterrupt:
    print("Stopping...")
    send_movement_command(0, 0, 0)  # Stop
    master.close()

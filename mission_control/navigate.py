from pymavlink import mavutil
import time

# --- User parameters ---
#connection_string = 'udpout:192.168.137.16:'  # Change as needed

home_waypoint_lat = 34.4038788
home_waypoint_lon = -119.696746
home_waypoint_alt = 2.73

waypoint_lat = 34.4039202  # Replace with your target latitude
waypoint_lon = -119.6968657   # Replace with your target longitude
waypoint_alt = 1.45         # Altitude in meters (relative to home)

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

# --- Monitor mission progress (optional) ---
while True:
    msg = master.recv_match(type='MISSION_CURRENT', blocking=True, timeout=10)
    if msg:
        print(f"Current mission item: {msg.seq}")
        if msg.seq == wp_seq:
            print("Arrived at target waypoint.")
            break
    else:
        print("Waiting for mission progress update...")

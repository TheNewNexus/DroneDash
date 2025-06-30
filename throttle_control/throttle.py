from pymavlink import mavutil
import time
 
# === VEHICLE A (CONTROLLER) SETUP ===
#vehicle_b_ip = '192.168.68.77'  # Vehicle B's IP
vehicle_b_ip = '0.0.0.0'  # Vehicle B's IP
control_conn = mavutil.mavlink_connection(f'udpout:{vehicle_b_ip}:14550')
 
# === MOVEMENT COMMAND FUNCTION ===
def send_body_velocity(vx, vy, vz, yaw_rate=0):
    """Send velocity command in body-frame coordinates (IMU relative)"""
    control_conn.mav.set_position_target_local_ned_send(
        int(time.time() * 1e3),  # Timestamp
        control_conn.target_system,
        control_conn.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # Body relative coordinates
        0b0000111111000111,  # Velocity components mask
        0, 0, 0,  # Position (ignored)
        vx, vy, vz,  # Velocity in m/s (forward, right, down)
        0, 0, 0,  # Acceleration (ignored)
        yaw_rate, 0  # Yaw rate, yaw rate acceleration
    )
 
# === MAIN CONTROL LOOP ===
try:
    while True:
        print("\n1: Forward | 2: Back | 3: Left | 4: Right | 5: Up | 6: Down | q: Quit")
        choice = input("Command: ").lower()
 
        if choice == '1': send_body_velocity(0.5, 0, 0)  # 0.5 m/s forward
        elif choice == '2': send_body_velocity(-0.5, 0, 0)
        elif choice == '3': send_body_velocity(0, -0.5, 0)
        elif choice == '4': send_body_velocity(0, 0.5, 0)
        elif choice == '5': send_body_velocity(0, 0, -0.5)  # Up in NED
        elif choice == '6': send_body_velocity(0, 0, 0.5)
        elif choice == 'q': break
 
except KeyboardInterrupt:
    print("\nStopping...")

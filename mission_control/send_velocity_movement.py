from pymavlink import mavutil
import time

# Connect to the Pixhawk (adjust serial port and baudrate as needed)
#master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=921600)
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')

# Wait for heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received")

# Set mode to GUIDED (required for movement commands)
master.set_mode('GUIDED')
time.sleep(1)

# Prepare velocity command (move forward in X axis, NED frame)
def send_forward_velocity(velocity=1.0, duration=1.0):
    """
    Move forward at 'velocity' m/s for 'duration' seconds.
    """
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

# Move forward for 1 second at 1 m/s
send_forward_velocity(velocity=1.0, duration=5.0)
print("Movement complete.")

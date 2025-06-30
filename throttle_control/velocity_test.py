from pymavlink import mavutil
import time

# Connect to vehicle
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')  # Adjust connection string
master.wait_heartbeat()

def send_command(command, params=[], confirmation=0):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        confirmation,
        *params
    )

# Arm vehicle (example)
send_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1, 0, 0, 0, 0, 0, 0])
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(f"Arming result: {msg.result}")

# Set mode to GUIDED (example)
mode_id = master.mode_mapping()['GUIDED']
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)

# Send movement command (example using COMMAND_INT)
while True:
    master.mav.command_int_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(5)

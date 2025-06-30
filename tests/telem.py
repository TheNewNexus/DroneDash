from pymavlink import mavutil

# Connect to Pixhawk via internal serial connection (TELEM2 <-> Jetson THS1)
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=921600)

# Wait for heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat from system {master.target_system} component {master.target_component}")

# Main loop
while True:
    try:
        # Check for incoming messages
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg:
            print(f"Received: {msg.get_type()}")  # Print message type
            
            # Example: Handle attitude data
            if msg.get_type() == 'ATTITUDE':
                print(f"Roll: {msg.roll:.2f} rad, Pitch: {msg.pitch:.2f} rad")
            
            # Example: Send RC override
            # Channels: [1-8], PWM: 1100-1900 (1500 center)
            # master.mav.rc_channels_override_send(
            #     master.target_system,
            #     master.target_component,
            #     *[65535]*8  # Set all channels to 65535 (no override)
            # )
            
            # Example: Send command (arm/disarm etc)
            # master.mav.command_long_send(
            #     master.target_system,
            #     master.target_component,
            #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            #     0,  # Confirmation
            #     1,  # Arm (1=arm, 0=disarm)
            #     0,0,0,0,0,0  # Parameters
            # )

    except KeyboardInterrupt:
        print("Exiting...")
        break
